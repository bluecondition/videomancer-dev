-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Girih v0.1.0 -- Kinetic.
--
-- Islamic star patterns by Hankin's "polygons in contact" method: from the midpoint of every tile edge two rays
-- leave at the contact angle; where rays meet they form stars.  The contact angle is each tile's state, so the
-- stars open and close in travelling waves while the lines stay joined at the shared edge midpoints.
--
--   element  a tile of pitch P (Square lattice), or the octagons and small squares of the 4.8.8 tiling (Octagon)
--   fold     the pixel is folded into the tile's fundamental domain: |x|, |y|, swap about the diagonal (and, in
--            Octagon, a reflection about the 22.5 degree line: (x + y, x - y) * 181 / 256).  There the pattern is ONE
--            ray from the edge midpoint: with u = |X - P/2| and Y the coordinate along the edge,
--   ray      d = Y * |sin a| - u * |cos a|   (a true distance: two 10 x 8 multiplies, quarter virtual px units)
--   strap    |d| < w; the sign of d separates star (tile centre side) from field; coverage = |d| - w over 2 px (SD 4)
--   theta    = Spread * index - phase;  index: Linear = column + row; Radial = max + min / 2 of (|column|, |row|);
--              the small squares of the octagon lattice sit half a tile off on both axes
--   a        = base (P12) - Depth * wave(theta) - source     (14 bits per turn; |sin|, |cos| fold it back)
--   out      = T(side, coverage): a per-field table, strap + (side colour - strap) * coverage / 8
--
--   K1  Scale    tile pitch 60 .. 480 virtual lines (exponential, glided)
--   K2  Shape    strap half-width 1.5 px .. 1.5 px + 3/32 pitch (SD: from 2.5 px)
--   K3  Depth    angle swing 0 .. 1/4 turn (square law)
--   K4  Rate     bipolar, centre = still; 60 s per cycle .. 2 cycles per second
--   K5  Colour   the 16 shared palettes  [K6 camera pass: was K6; the old K5 Spread, the phase step per tile
--                (square law, 0 .. 1/2 cycle), is fixed at its default 330]
--   K6  Camera   how far the held source closes each tile's angle: linear gain, 0 = none (the black-input
--                picture), 20 % = the v0.1.0 strength (Video x6, white = 29 degrees), 100 % = five times
--                (2026-11-15 +25 %; Osc keeps its 16 / 6 of the camera's gain).  Glided per field, applied to
--                the smeared sample by one 8 x 11 multiply (8 x 5 and 8 x 6 halves, then their sum) before the hold RAM
--   S7  Wave     Smooth / Snap
--   S8  Field    Linear / Radial
--   S9  Lattice  Square / Octagon
--   S10 Look     Lines (straps in A on the ground) / Filled (stars A, field B, straps accent)
--   S11 Source   Video / Osc  (luma held per tile -> contact angle)
--   P12 ANGLE    base contact angle: 81.6 degrees (thin sharp stars) .. 0 .. folding back to 19.6 degrees
--
-- Built on the Louver chassis:
--   [A] timing ID -> virtual-1080 steps, vblank zone, width, knob glides, per-field sequencer (serial multiplier)
--   [B] generated avid
--   [C] source field: Osc lattice, luma smear, per-tile hold
--   [D] tile accumulators, fold, wave engine
--   [E] the look: ray distance, coverage, blend table
--   [F] output conditioning: luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1] with resync, luma 13/16, blanking gate
-- One latency (C_LAT = 36) in every mode.  Colours in the hardware convention (u = Cr, v = Cb).

-- K6 camera pass 2026-11-15: K6 = Camera (camera strength, new); K5 = Colour (was K6); K5 Spread fixed at 330 (C_KFIX_5).
-- 2026-11-15: K6 camera top +25 % (Ron)
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.girih_pkg.all;
use work.all;

architecture girih of program_top is
    constant C_KFIX_5 : std_logic_vector(9 downto 0) := std_logic_vector(to_unsigned(330, 10));   -- K6 camera pass: fixed knob
    constant C_LAT      : integer := 36;      -- = processing_delay_clks
    constant C_SIM_SNAP : boolean := false;   -- sim-only copies set this: knob glides jump straight to the knob

    -- [A] virtual-1080 steps from the timing ID: y step per field line (Q8), x step in half virtual px
    function f_vstep_y(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(1138, 12);
            when 4          => return to_unsigned(576, 12);
            when 8          => return to_unsigned(960, 12);
            when 12         => return to_unsigned(480, 12);
            when 5 | 6 | 14 => return to_unsigned(384, 12);
            when 1 | 2 | 10 => return to_unsigned(512, 12);
            when others     => return to_unsigned(256, 12);
        end case;
    end function;
    function f_vstep_x(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0 | 4 | 8 | 12 => return to_unsigned(4, 3);
            when 5 | 6 | 14     => return to_unsigned(3, 3);
            when others         => return to_unsigned(2, 3);
        end case;
    end function;
    function f_tri7(v : unsigned(7 downto 0)) return unsigned is
    begin
        if v(7) = '1' then return not v(6 downto 0); else return v(6 downto 0); end if;
    end function;
    -- gated operands: every multiply in this file is a sum of these (one adder per two operand bits per stage)
    function f_gu(x : unsigned; b : std_logic; w : natural) return unsigned is
        variable r : unsigned(w - 1 downto 0) := (others => '0');
    begin
        if b = '1' then r := resize(x, w); end if;
        return r;
    end function;
    function f_gs(x : signed; b : std_logic; w : natural) return signed is
        variable r : signed(w - 1 downto 0) := (others => '0');
    begin
        if b = '1' then r := resize(x, w); end if;
        return r;
    end function;

    attribute no_rw_check : boolean;

    -- ROMs (one read port each, registered address and data, then a fabric register)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_w_a  : unsigned(9 downto 0) := (others => '0');
    signal r_w_q  : unsigned(15 downto 0) := (others => '0');
    signal r_cs_a : unsigned(8 downto 0) := (others => '0');
    signal r_cs_q : unsigned(15 downto 0) := (others => '0');
    signal r_pl_a : unsigned(7 downto 0) := (others => '0');
    signal r_pl_q : std_logic_vector(31 downto 0) := (others => '0');

    -- [C] source hold: one smeared luma sample per tile column, written on the last line before each tile row
    type t_r16 is array (0 to 255) of std_logic_vector(15 downto 0);
    signal sl : t_r16 := (others => (others => '0'));
    attribute no_rw_check of sl : signal is true;
    signal sl_ra, sl_wa : unsigned(7 downto 0) := (others => '0');
    signal sl_q, sl_wd  : std_logic_vector(15 downto 0) := (others => '0');
    signal sl_we : std_logic := '0';
    -- delay rings (write at rc + d, read at rc, RAM register, fabric register: d + 2 stages)
    type t_r32 is array (0 to 255) of std_logic_vector(31 downto 0);
    signal vr : t_r32 := (others => (others => '0'));          -- folded coordinates {u, Y}: stage 9 -> 20
    attribute no_rw_check of vr : signal is true;
    signal vr_d, vr_q : std_logic_vector(31 downto 0) := (others => '0');
    signal cr : t_r16 := (others => (others => '0'));          -- chroma 8 back for the running 8-sums
    attribute no_rw_check of cr : signal is true;
    signal cr_q, cr_f : std_logic_vector(15 downto 0) := (others => '0');
    -- [E] blend table, refilled every field: T(side, cov) = strap + (side colour - strap) cov / 8
    -- {Y 10 bits, U 8, V 8}; address = side & cov
    signal tt : t_r32 := (others => (others => '0'));
    attribute no_rw_check of tt : signal is true;
    signal tt_ra, tt_wa : unsigned(7 downto 0) := (others => '0');
    signal tt_q, tt_wd  : std_logic_vector(31 downto 0) := (others => '0');
    signal tt_we : std_logic := '0';
    signal t_run, t_side : std_logic := '0';
    signal t_k   : unsigned(3 downto 0) := (others => '0');
    signal t_ey  : signed(13 downto 0) := (others => '0');
    type t_s12 is array (1 to 2) of signed(11 downto 0);
    signal t_ec  : t_s12 := (others => (others => '0'));
    type t_r4 is array (0 to 255) of std_logic_vector(3 downto 0);
    signal sr : t_r4 := (others => (others => '1'));           -- syncs: C_LAT
    attribute no_rw_check of sr : signal is true;
    signal sr_q, sr_f : std_logic_vector(3 downto 0) := (others => '1');
    signal rc   : unsigned(7 downto 0) := (others => '0');

    -- [A] timing / zone / width
    signal s_tid  : std_logic_vector(3 downto 0) := (others => '0');
    signal s_ys   : unsigned(11 downto 0) := to_unsigned(256, 12);
    signal s_xsr  : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_xs   : unsigned(2 downto 0) := to_unsigned(2, 3);
    signal s_sd   : std_logic := '0';
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);

    -- knobs: glided (Q3), raw rate, switches, palette
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(650 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(256 * 8, 13);
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    -- K6 Camera: its own glide (Q3, 1/8 of the gap per field, +-4 deadband) and the source gain (x / 16)
    signal s_gk6 : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);
    signal s_k6d : signed(14 downto 0) := (others => '0');
    signal s_cm  : unsigned(10 downto 0) := to_unsigned(240, 11);   -- K6 top +25 %: 11 bits (Osc up to 1278)
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_snap, s_rad, s_oct, s_fill, s_src : std_logic := '0';
    signal s_pal : unsigned(3 downto 0) := to_unsigned(2, 4);
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- field serial multiplier (24 iterations), serial shifter, sequencer scratch
    signal f_a, f_acc, z_fa : unsigned(47 downto 0) := (others => '0');
    signal f_b, z_fb : unsigned(23 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_e   : unsigned(10 downto 0) := (others => '0');
    signal z_sh  : unsigned(2 downto 0) := (others => '0');
    signal z_shn : unsigned(2 downto 0) := (others => '0');
    signal z_sht : std_logic := '0';
    signal z_mt  : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_loc : signed(18 downto 0) := (others => '0');
    signal z_k   : signed(7 downto 0) := (others => '0');
    signal z_nrun, z_first : std_logic := '0';

    -- per-field constants
    signal c_pw   : unsigned(19 downto 0) := to_unsigned(61440 * 8, 20);   -- pitch, Q11 (c_p = Q8 virtual lines)
    signal c_p    : unsigned(16 downto 0);
    signal c_ph2  : unsigned(9 downto 0);                               -- pitch / 2, Q2
    signal c_w    : unsigned(8 downto 0) := to_unsigned(40, 9);        -- strap half-width, Q2
    signal c_pd   : unsigned(10 downto 0) := to_unsigned(600, 11);     -- pitch / sqrt 2, Q2
    signal c_d    : unsigned(7 downto 0) := (others => '0');           -- depth, 255 = 1/4 turn
    signal c_s    : unsigned(22 downto 0) := (others => '0');          -- spread per tile, Q24 cycles
    signal c_hsp, c_hsn : signed(19 downto 0) := (others => '0');      -- +- half a tile's spread, Q14
    signal c_bang : unsigned(13 downto 0) := to_unsigned(2560, 14);    -- base contact angle, 14 bits per turn
    signal c_xst  : unsigned(9 downto 0) := to_unsigned(256, 10);      -- x step per pixel, Q8
    signal c_yst  : unsigned(11 downto 0) := to_unsigned(256, 12);     -- y step per line, Q8
    signal c_xmp, c_x2mp, c_ymp : signed(18 downto 0) := (others => '0');
    signal c_wrap0 : std_logic := '0';
    signal c_xloc0 : unsigned(16 downto 0) := (others => '0');         -- the line's first pixel: offset in its tile,
    signal c_xk0   : signed(7 downto 0) := (others => '0');            -- column number,
    signal c_xsk0  : signed(27 downto 0) := (others => '0');           -- Spread * column
    signal c_ph   : unsigned(25 downto 0) := (others => '0');          -- wave phase, Q26 cycles
    signal c_dph  : unsigned(21 downto 0) := (others => '0');
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u8  is array (1 to 2) of unsigned(7 downto 0);
    type t_s9  is array (1 to 2) of signed(8 downto 0);
    -- palette: luma 10 bits, chroma 8 bits (raw)
    signal c_gy, c_ay, c_by, c_cy, c_sy : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal c_gc, c_ac, c_bc, c_cc, c_sc : t_u8 := (others => to_unsigned(128, 8));
    signal c_d0y, c_d1y : signed(10 downto 0) := (others => '0');      -- side colour - strap colour
    signal c_d0c, c_d1c : t_s9 := (others => (others => '0'));

    -- [B] generated avid
    signal g_hsq, g_hsr, g_avq, g_avr : std_logic := '0';
    signal g_ph, g_arm, g_par, g_sel, g_avid : std_logic := '0';
    signal g_pc  : unsigned(3 downto 0) := to_unsigned(8, 4);
    signal g_rsr : std_logic_vector(2 downto 0) := (others => '0');
    signal g_cnt : unsigned(11 downto 0) := (others => '0');
    signal g_wm1 : unsigned(11 downto 0) := to_unsigned(1919, 12);
    signal g_start : std_logic := '0';

    -- [C] source
    signal s_ox, s_oy : unsigned(15 downto 0) := (others => '0');
    signal s_ot  : unsigned(7 downto 0) := (others => '0');
    signal s_oxs : unsigned(8 downto 0) := to_unsigned(102, 9);
    signal s_oys : unsigned(9 downto 0) := to_unsigned(121, 10);
    signal s_sv  : unsigned(7 downto 0) := (others => '0');
    signal s_sav, s_savd, s_srow, s_wbank : std_logic := '0';
    signal s_ext : unsigned(5 downto 0) := (others => '0');
    signal s_y1 : unsigned(13 downto 0) := (others => '0');
    signal s_ygl : unsigned(12 downto 0) := (others => '0');          -- smeared sample x K6 gain, two halves
    signal s_ygh : unsigned(13 downto 0) := (others => '0');          -- (K6 top +25 %: high half 8 x 6)
    signal s_yg : unsigned(13 downto 0) := (others => '0');           -- the source term (14 bits per turn)

    -- [D] tile accumulators: line registers (the NEXT line's start values), pixel accumulators (stage 1)
    signal ls_sk  : signed(27 downto 0) := (others => '0');
    signal ls_loc : unsigned(16 downto 0) := (others => '0');
    signal ls_n1  : unsigned(16 downto 0) := (others => '0');
    signal ls_n2  : signed(18 downto 0) := (others => '0');
    signal a_sk   : signed(27 downto 0) := (others => '0');
    signal a_loc  : unsigned(16 downto 0) := (others => '0');
    signal a_wrap, a_nw : std_logic := '0';
    signal l_yloc : unsigned(16 downto 0) := (others => '0');          -- this line: offset in its tile row,
    signal l_ysk  : signed(27 downto 0) := (others => '0');            -- Spread * row
    signal l_ty   : signed(11 downto 0) := (others => '0');
    signal l_ay   : unsigned(10 downto 0) := (others => '0');
    signal l_syp, ls_top, l_top : std_logic := '0';
    signal s_nw   : std_logic_vector(2 to 18) := (others => '0');
    -- fold
    signal q2_tx  : signed(11 downto 0) := (others => '0');
    signal q2_sk, q3_sk, q4_sk, q5_sk : signed(19 downto 0) := (others => '0');
    signal q3_ax, q4_ax : unsigned(10 downto 0) := (others => '0');
    signal q3_sxp, q4_sxp, q5_sxp : std_logic := '0';
    signal q4_s, q5_s : unsigned(11 downto 0) := (others => '0');
    signal q4_dd  : signed(11 downto 0) := (others => '0');
    signal q5_hi, q5_lo, q5_df, q6_hi, q6_lo, q7_hi, q7_lo, q7_yr : unsigned(10 downto 0) := (others => '0');
    signal q6_sa, q6_sb : unsigned(19 downto 0) := (others => '0');
    signal q6_da, q6_db : unsigned(18 downto 0) := (others => '0');
    signal q7_xr, q8_x : unsigned(11 downto 0) := (others => '0');
    signal q8_y, q9_y, q9_u : unsigned(9 downto 0) := (others => '0');
    signal s_sq   : std_logic_vector(5 to 15) := (others => '0');
    -- wave
    signal q6_ex, q6_ey : signed(19 downto 0) := (others => '0');
    signal q7_aex, q7_aey : unsigned(18 downto 0) := (others => '0');
    signal q7_lin, q8_lin, q8_nph, q9_xa, q9_xb, q10_th : unsigned(13 downto 0) := (others => '0');
    signal q8_ge  : std_logic := '0';
    signal q8_aex, q8_aey : unsigned(14 downto 0) := (others => '0');
    signal s_neg  : std_logic_vector(11 to 16) := (others => '0');
    signal q13_w  : unsigned(9 downto 0) := (others => '0');
    signal q14_p0, q14_p1, q14_p2, q14_p3 : unsigned(11 downto 0) := (others => '0');
    signal q15_qa, q15_qb : unsigned(13 downto 0) := (others => '0');
    signal q15_src : unsigned(13 downto 0) := (others => '0');   -- the held source term, K6 gain applied
    signal q16_aw : unsigned(11 downto 0) := (others => '0');
    signal q16_st, q17_al : unsigned(13 downto 0) := (others => '0');
    signal k13, k14, k15 : unsigned(7 downto 0) := (others => '0');

    -- [E] the look
    signal q20_cos, q20_sin : unsigned(7 downto 0) := (others => '0');
    signal q20_u, q20_y : unsigned(9 downto 0) := (others => '0');
    signal q21_a0, q21_a1, q21_a2, q21_a3, q21_b0, q21_b1, q21_b2, q21_b3 : unsigned(11 downto 0) := (others => '0');
    signal q22_a01, q22_a23, q22_b01, q22_b23 : unsigned(13 downto 0) := (others => '0');
    signal q23_pa, q23_pb : unsigned(17 downto 0) := (others => '0');
    signal q24_d  : signed(10 downto 0) := (others => '0');
    signal q25_e  : signed(11 downto 0) := (others => '0');
    signal q25_sd, q26_sd : std_logic := '0';
    signal q26_cov : unsigned(3 downto 0) := (others => '0');
    signal q29_y  : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal q29_c  : t_u8 := (others => to_unsigned(128, 8));

    -- [F] output conditioning
    signal s_ysr  : t_u10(0 to 5) := (others => to_unsigned(64, 10));
    signal s_bu, s_bv, s_bud, s_bvd : unsigned(10 downto 0) := to_unsigned(1024, 11);
    signal s_bah  : std_logic_vector(7 downto 0) := (others => '0');
    signal s_y31, s_y32, s_y33 : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal s_u31, s_v31, s_u32, s_v32, s_u33, s_v33 : unsigned(9 downto 0) := to_unsigned(512, 10);

    type t_bit_shift is array (0 to C_LAT - 1) of std_logic;
    signal s_gav_sr   : t_bit_shift := (others => '0');   -- s_gav_sr(k) = generated avid of the stage-(k+2) pixel

begin

    c_p <= c_pw(19 downto 3);
    c_ph2 <= c_pw(19 downto 10);

    ---------------------------------------------------------------------------
    -- Block RAM: ROMs, hold, delay rings (one write statement each; none ever
    -- reads the address it is writing)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_GH_EXP2(to_integer(r_ex_a));
            r_w_q  <= C_GH_SIN(to_integer(r_w_a));
            r_cs_q <= C_GH_CS(to_integer(r_cs_a));
            r_pl_q <= C_GH_PAL(to_integer(r_pl_a));
        end if;
    end process p_rom;

    p_sl : process(clk)
    begin
        if rising_edge(clk) then
            sl_q <= sl(to_integer(sl_ra));
            if sl_we = '1' then sl(to_integer(sl_wa)) <= sl_wd; end if;
        end if;
    end process p_sl;

    p_tt : process(clk)
    begin
        if rising_edge(clk) then
            tt_q <= tt(to_integer(tt_ra));
            if tt_we = '1' then tt(to_integer(tt_wa)) <= tt_wd; end if;
        end if;
    end process p_tt;

    p_ring : process(clk)
    begin
        if rising_edge(clk) then
            vr(to_integer(rc + 9)) <= vr_d;
            vr_q <= vr(to_integer(rc));
            cr(to_integer(rc + 6)) <= std_logic_vector(q29_c(1)) & std_logic_vector(q29_c(2));
            cr_q <= cr(to_integer(rc));
            cr_f <= cr_q;
            sr(to_integer(rc + (C_LAT - 2))) <= '1' & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            sr_q <= sr(to_integer(rc));
            sr_f <= sr_q;
            rc <= rc + 1;
        end if;
    end process p_ring;
    vr_d <= x"000" & std_logic_vector(q9_u) & std_logic_vector(q9_y);

    ---------------------------------------------------------------------------
    -- [A] Controls, vblank zone, glides, per-field sequencer (one small
    -- operation per zone clock; quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_d  : signed(10 downto 0);
        variable v_x8 : unsigned(9 downto 0);
        variable v_l  : signed(18 downto 0);
        variable v_b  : unsigned(25 downto 0);
        variable v_a27 : unsigned(26 downto 0);
        variable v_g9 : unsigned(16 downto 0);
        variable v_a16 : unsigned(15 downto 0);
        variable v_sy, v_0y, v_1y : unsigned(9 downto 0);
        variable v_sc, v_0c, v_1c : t_u8;
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);

            -- vblank zone: no avid for 1024 clocks
            if data_in.avid = '1' then
                s_idle <= (others => '0');
            elsif s_idle /= 1023 then
                s_idle <= s_idle + 1;
            end if;
            if s_idle = 1023 then s_zone <= '1'; else s_zone <= '0'; end if;
            s_zone_d <= s_zone;
            s_zgo <= s_zone and not s_zone_d;
            if s_zone = '0' then
                s_zc <= (others => '0');
            elsif s_zc /= 2047 then
                s_zc <= s_zc + 1;
            end if;

            -- width: source avid pixels per line, max over the field
            s_avq <= data_in.avid;
            if data_in.avid = '1' then
                s_hcnt <= s_hcnt + 1;
            elsif s_avq = '1' then
                if s_hcnt > s_wmax then s_wmax <= s_hcnt; end if;
                s_hcnt <= (others => '0');
            end if;

            -- Rate: bipolar with a +-8 deadband
            s_k4 <= unsigned(registers_in(3));
            v_s := signed(resize(s_k4, 11)) - 512;
            s_vneg <= v_s(10);
            if v_s > -9 and v_s < 9 then
                v_m := (others => '0');
            elsif v_s(10) = '1' then
                v_m := unsigned(not v_s(9 downto 0)) - 7;     -- |v| - 8
            else
                v_m := unsigned(v_s(9 downto 0)) - 8;
            end if;
            s_vm <= v_m(8 downto 0);

            if s_zgo = '1' then
                s_w <= s_wmax; s_wmax <= (others => '0');
                s_snap <= registers_in(6)(0);
                s_rad  <= registers_in(6)(1);
                s_oct  <= registers_in(6)(2);
                s_fill <= registers_in(6)(3);
                s_src  <= registers_in(6)(4);
                s_xs   <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9); s_sd <= '1';
                    when 3      => s_oxs <= to_unsigned(153, 9); s_sd <= '0';
                    when others => s_oxs <= to_unsigned(102, 9); s_sd <= '0';
                end case;
                -- Colour: 16 palettes with hysteresis
                v_d := signed(resize(unsigned(registers_in(4)), 11)) - signed(resize(s_pal & "100000", 11));
                if v_d > 40 or v_d < -40 then s_pal <= unsigned(registers_in(4)(9 downto 6)); end if;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= resize(shift_right(resize(s_ys, 19) * to_unsigned(121, 7), 8), 10);

            -- shared glide for Scale and Angle (1/8 of the gap per field, +-2 deadband): four clocks per channel
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        if z_gc(0) = '0' then s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                        else s_gm <= s_gp; s_gr <= unsigned(registers_in(7)); end if;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>
                        if C_SIM_SNAP then
                            null;
                        elsif s_gd > 16 or s_gd < -16 then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp <= "10";
                    when others =>
                        if z_gc(0) = '0' then s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        else s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0)); end if;
                        z_gp <= "00";
                        if z_gc = 1 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products: slot k loads at zc = 32 + 32 k, 24 iterations, stored at 60 + 32 k.  The slot
            -- number is decoded at 30 + 32 k and the operands pre-muxed at 31 + 32 k (no zc decode on the
            -- multiplier's path).
            z_pre <= '0';
            if s_zone = '1' and s_zc(4 downto 0) = 30 and s_zc(10 downto 9) = "00" then
                z_sel <= s_zc(8 downto 5); z_pre <= '1';
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                z_fa <= (others => '0'); z_fb <= (others => '0');
                case to_integer(z_sel) is
                    when 0  => z_fa(12 downto 0) <= s_g1;                 z_fb <= to_unsigned(6150, 24);      -- pitch exponent
                    when 2  => z_fa(16 downto 0) <= c_p;                  z_fb(9 downto 0) <= unsigned(registers_in(1)) - shift_right(unsigned(registers_in(1)), 2);  -- strap width
                    when 3  =>                                                                                -- depth^2
                        z_fa(12 downto 3) <= unsigned(registers_in(2)); z_fb(12 downto 3) <= unsigned(registers_in(2));
                    when 4  => z_fa(16 downto 0) <= c_p;                  z_fb <= to_unsigned(181, 24);       -- pitch / sqrt 2
                    when 7  =>                                                                                -- spread^2
                        z_fa(12 downto 3) <= unsigned(C_KFIX_5); z_fb(12 downto 3) <= unsigned(C_KFIX_5);
                    when 12 => z_fa(8 downto 0)  <= s_vm;                 z_fb <= to_unsigned(900, 24);       -- rate exponent
                    when others => z_fa(15 downto 0) <= z_mt;             z_fb <= to_unsigned(1193046, 24);   -- rate (14)
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= z_fa; f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(46 downto 0) & '0';
                f_b <= '0' & f_b(23 downto 1);
            end if;

            -- serial shifter for the 2^n scalings (no barrels)
            if z_shn /= 0 then
                z_shn <= z_shn - 1;
                if z_sht = '1' then c_pw <= c_pw(18 downto 0) & '0';
                else c_dph <= c_dph(20 downto 0) & '0'; end if;
            end if;

            -- tile origin: tile 0 is centred on the screen centre; step back to the first pixel / line
            if z_nrun = '1' then
                if z_loc(18) = '1' then
                    z_loc <= z_loc + signed(resize(c_p, 19));
                    z_k   <= z_k - 1;
                else
                    z_nrun <= '0';
                end if;
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- K6 Camera: glide (1/8 of the gap per field, +-4 deadband), then the source gain x / 16:
                    -- Video = 3 K6 / 8 (96 at 25 % = v0.1.0's x6), Osc = K6 (256 at 25 % = v0.1.0's x16)
                    when 10 => s_k6d <= signed(resize(unsigned(registers_in(5)) & "000", 15)) - signed(resize(s_gk6, 15));
                    when 11 =>
                        if C_SIM_SNAP then
                            null;
                        elsif s_k6d > 32 or s_k6d < -32 then
                            s_k6d <= shift_right(s_k6d, 3);
                        else
                            s_k6d <= (others => '0');
                        end if;
                    when 12 => s_gk6 <= unsigned(signed(s_gk6) + s_k6d(12 downto 0));
                    -- K6 top +25 % (2026-11-15): Video 15 K6 / 32 = (2 gk6 - gk6 / 8) / 32 (<= 479, was 3 K6 / 8),
                    -- Osc 5 K6 / 4 = 5 gk6 / 32 (<= 1278, was K6); gk6 = 8 K6 (Q3), so both fit 16 bits
                    when 13 =>
                        if s_src = '1' then v_a16 := resize(s_gk6 & "00", 16) + resize(s_gk6, 16);
                        else v_a16 := resize(s_gk6 & '0', 16) - resize(s_gk6(12 downto 3), 16); end if;
                        s_cm <= v_a16(15 downto 5);
                    -- wave phase (last field's step): one adder, add or subtract
                    when 20 =>
                        v_b := resize(c_dph, 26);
                        if s_vneg = '1' then v_b := not v_b; end if;
                        v_a27 := (c_ph & '1') + (v_b & s_vneg);
                        c_ph <= v_a27(26 downto 1);
                    -- palette: ground, A, B, accent (chroma kept to 8 bits)
                    when 22 => r_pl_a <= "00" & s_pal & "00";
                    when 25 =>
                        c_gy <= unsigned(r_pl_q(31 downto 22)); c_gc(1) <= unsigned(r_pl_q(21 downto 14)); c_gc(2) <= unsigned(r_pl_q(11 downto 4));
                        r_pl_a <= "00" & s_pal & "01";
                    when 28 =>
                        c_ay <= unsigned(r_pl_q(31 downto 22)); c_ac(1) <= unsigned(r_pl_q(21 downto 14)); c_ac(2) <= unsigned(r_pl_q(11 downto 4));
                        r_pl_a <= "00" & s_pal & "10";
                    when 31 =>
                        c_by <= unsigned(r_pl_q(31 downto 22)); c_bc(1) <= unsigned(r_pl_q(21 downto 14)); c_bc(2) <= unsigned(r_pl_q(11 downto 4));
                        r_pl_a <= "00" & s_pal & "11";
                    when 34 =>
                        c_cy <= unsigned(r_pl_q(31 downto 22)); c_cc(1) <= unsigned(r_pl_q(21 downto 14)); c_cc(2) <= unsigned(r_pl_q(11 downto 4));
                    -- Look: Lines = straps in A on the ground; Filled = stars A, field B, straps accent
                    when 36 =>
                        if s_fill = '1' then
                            v_sy := c_cy; v_sc := c_cc; v_0y := c_ay; v_0c := c_ac; v_1y := c_by; v_1c := c_bc;
                        else
                            v_sy := c_ay; v_sc := c_ac; v_0y := c_gy; v_0c := c_gc; v_1y := c_gy; v_1c := c_gc;
                        end if;
                        c_sy <= v_sy; c_sc <= v_sc;
                        c_d0y <= signed(resize(v_0y, 11)) - signed(resize(v_sy, 11));
                        c_d1y <= signed(resize(v_1y, 11)) - signed(resize(v_sy, 11));
                        for i in 1 to 2 loop
                            c_d0c(i) <= signed(resize(v_0c(i), 9)) - signed(resize(v_sc(i), 9));
                            c_d1c(i) <= signed(resize(v_1c(i), 9)) - signed(resize(v_sc(i), 9));
                        end loop;

                    -- pitch: e = k1 * 3 octaves (Q8), P = 60 * 2^e
                    when 60 => z_e <= f_acc(26 downto 16);
                    when 61 => r_ex_a <= z_e(7 downto 0); z_sh <= z_e(10 downto 8);
                    when 63 => z_mt <= r_ex_q;
                    when 64 =>
                        c_pw <= resize(z_mt & '0', 20) - resize(z_mt(15 downto 3), 20);
                        z_shn <= z_sh + 1; z_sht <= '1';
                    when 80 =>
                        v_x8 := resize(s_xs, 10);
                        c_xst <= shift_left(v_x8, 7); c_yst <= s_ys;
                    when 81 =>
                        c_xmp  <= signed(resize(c_xst, 19)) - signed(resize(c_p, 19));
                        c_x2mp <= signed(resize(c_xst & '0', 19)) - signed(resize(c_p, 19));
                        c_ymp  <= signed(resize(c_yst, 19)) - signed(resize(c_p, 19));
                    when 82 =>                       -- base contact angle = 3712 - 9/16 angle knob (Q3)
                        v_g9 := resize(s_gp & "000", 17) + resize(s_gp, 17);
                        c_bang <= to_unsigned(3712, 14) - resize(v_g9(16 downto 4), 14);
                    when 124 =>                      -- strap half-width: 1.5 px (SD: 2.5 px, a full SD line each side) + Shape
                        if s_sd = '1' then c_w <= to_unsigned(10, 9) + resize(f_acc(26 downto 19), 9);
                        else c_w <= to_unsigned(6, 9) + resize(f_acc(26 downto 19), 9); end if;
                    when 156 => c_d <= f_acc(25 downto 18);       -- depth = (k3 / 1023)^2 * 1/4 turn
                    when 188 => c_pd <= f_acc(24 downto 14);
                    when 284 => c_s <= f_acc(25 downto 3);
                    when 300 =>
                        c_hsp <= signed(resize(c_s(22 downto 11), 20));
                        c_hsn <= - signed(resize(c_s(22 downto 11), 20));
                    -- rate: 2^(m * 6.9 / 503) / 3600 cycles per field
                    when 444 => z_e <= f_acc(18 downto 8);
                    when 445 => r_ex_a <= z_e(7 downto 0); z_sh <= z_e(10 downto 8);
                    when 447 => z_mt <= r_ex_q;
                    when 508 =>
                        if s_vm = 0 then c_dph <= (others => '0'); else c_dph <= resize(f_acc(36 downto 21), 22); end if;
                        z_shn <= z_sh; z_sht <= '0';
                    -- origin along x, then along y (p_acc keeps Spread * tile number in step)
                    when 550 =>
                        v_l := signed(resize(c_p(16 downto 1), 19));
                        if s_sd = '1' then z_loc <= v_l - 184320;         -- 720 px
                        else z_loc <= v_l - 245760; end if;               -- 960 px
                        z_k <= (others => '0');
                        z_nrun <= '1';
                    when 640 =>
                        c_xloc0 <= unsigned(z_loc(16 downto 0)); c_xk0 <= z_k; c_xsk0 <= ls_sk;
                        if z_loc + c_xmp >= 0 then c_wrap0 <= '1'; else c_wrap0 <= '0'; end if;
                    when 650 =>
                        v_l := signed(resize(c_p(16 downto 1), 19));
                        z_loc <= v_l - 138240;                            -- 540 lines
                        z_k <= (others => '0');
                        z_nrun <= '1';
                    when 740 => z_first <= '1';
                    when others => null;
                end case;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
            if g_start = '1' then z_first <= '0'; end if;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Blend table fill (zone, zc 40 .. 60; adders only).  For side = 0, 1 and
    -- cov = 0 .. 8:  T = (8 strap + cov (side colour - strap)) / 8
    ---------------------------------------------------------------------------
    p_fill : process(clk)
    begin
        if rising_edge(clk) then
            tt_we <= '0';
            if s_zone = '1' and s_zc = 40 then
                t_run <= '1'; t_side <= '0'; t_k <= (others => '0');
                t_ey <= signed(resize(c_sy & "000", 14));
                for i in 1 to 2 loop t_ec(i) <= signed(resize(c_sc(i) & "000", 12)); end loop;
            elsif t_run = '1' then
                tt_we <= '1'; tt_wa <= "000" & t_side & t_k;
                tt_wd <= "000000" & std_logic_vector(t_ey(12 downto 3)) & std_logic_vector(t_ec(1)(10 downto 3)) & std_logic_vector(t_ec(2)(10 downto 3));
                if t_k = 8 then
                    t_k <= (others => '0');
                    t_ey <= signed(resize(c_sy & "000", 14));
                    for i in 1 to 2 loop t_ec(i) <= signed(resize(c_sc(i) & "000", 12)); end loop;
                    t_side <= '1';
                    if t_side = '1' then t_run <= '0'; end if;
                else
                    t_k <= t_k + 1;
                    if t_side = '0' then
                        t_ey <= t_ey + resize(c_d0y, 14);
                        for i in 1 to 2 loop t_ec(i) <= t_ec(i) + resize(c_d0c(i), 12); end loop;
                    else
                        t_ey <= t_ey + resize(c_d1y, 14);
                        for i in 1 to 2 loop t_ec(i) <= t_ec(i) + resize(c_d1c(i), 12); end loop;
                    end if;
                end if;
            end if;
        end if;
    end process p_fill;

    ---------------------------------------------------------------------------
    -- [B] Generated avid (parity-locked start 3/4 clocks after the source's
    -- avid rise, field-latched width)
    ---------------------------------------------------------------------------
    p_gavid : process(clk)
    begin
        if rising_edge(clk) then
            g_hsq <= data_in.hsync_n;
            g_hsr <= g_hsq and not data_in.hsync_n;
            g_avq <= data_in.avid;
            g_avr <= data_in.avid and not g_avq;

            if g_hsr = '1' then
                g_ph  <= '0';
                g_arm <= '1';
            else
                g_ph <= not g_ph;
            end if;
            g_rsr <= g_rsr(1 downto 0) & '0';
            if g_avr = '1' and g_arm = '1' then
                g_arm    <= '0';
                g_rsr(0) <= '1';
                g_sel    <= g_ph xor g_par;
                if g_ph = '1' then
                    if g_pc /= 15 then g_pc <= g_pc + 1; end if;
                else
                    if g_pc /= 0 then g_pc <= g_pc - 1; end if;
                end if;
            end if;
            if g_pc = 15 then g_par <= '1';
            elsif g_pc = 0 then g_par <= '0'; end if;

            if g_start = '1' then
                g_avid <= '1';
                g_cnt  <= (others => '0');
            elsif g_avid = '1' then
                g_cnt <= g_cnt + 1;
                if g_cnt = g_wm1 or g_hsr = '1' then
                    g_avid <= '0';
                end if;
            end if;
        end if;
    end process p_gavid;
    g_start <= '1' when (g_sel = '0' and g_rsr(1) = '1') or (g_sel = '1' and g_rsr(2) = '1') else '0';

    ---------------------------------------------------------------------------
    -- [C] Source field.  Osc: a two-axis triangle lattice (640 x 540 virtual
    -- px) drifting diagonally.  The luma (or the lattice) is smeared along x by
    -- a 1/64 filter; one sample per tile column is kept on the last line before
    -- each tile row (p_wave), so every tile holds one value and stays rigid.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_tx, v_ty : unsigned(6 downto 0);
        variable v_s  : unsigned(7 downto 0);
        variable v_o  : unsigned(7 downto 0);
        variable v_l  : unsigned(7 downto 0);
        variable v_x  : unsigned(13 downto 0);
        variable v_d1 : signed(14 downto 0);
        variable v_g18 : unsigned(17 downto 0);
    begin
        if rising_edge(clk) then
            -- oscillator
            if data_in.avid = '1' then
                s_ox <= s_ox + s_oxs;
            elsif s_avq = '1' then
                s_ox <= shift_left(resize(s_ot, 16), 7);                                      -- ot * 128
                s_oy <= s_oy + s_oys;
            end if;
            if s_zone = '1' then s_oy <= shift_left(resize(s_ot, 16), 6) + shift_left(resize(s_ot, 16), 5); end if;  -- ot * 96
            if s_zgo = '1' then s_ot <= s_ot + 1; end if;
            v_tx := f_tri7(s_ox(15 downto 8)); v_ty := f_tri7(s_oy(15 downto 8));
            v_s := resize(v_tx, 8) + resize(v_ty, 8);
            v_o := v_s - shift_right(v_s, 3);

            -- input sample: source value (8 bits), held through blanking
            s_sav <= data_in.avid; s_savd <= s_sav;
            if data_in.avid = '1' then
                if s_src = '1' then
                    s_sv <= v_o;
                else
                    v_l := unsigned(data_in.y(9 downto 2));
                    if v_l > 16 then s_sv <= v_l - 16; else s_sv <= (others => '0'); end if;
                end if;
            end if;

            -- smear; it runs 56 px past the line end
            if s_sav = '0' and s_savd = '1' then
                s_ext <= to_unsigned(56, 6);
            elsif s_ext /= 0 then
                s_ext <= s_ext - 1;
            end if;
            v_x := s_sv & "000000";
            if s_sav = '1' and s_savd = '0' then
                s_y1 <= v_x;
                s_srow <= z_first or not ls_n2(18);     -- line 0, or the last line before a new tile row
                s_wbank <= z_first and ls_n2(18);       -- line 0 feeds the top row's own bank (read from the next field on)
            elsif s_sav = '1' or s_ext /= 0 then
                v_d1 := signed(resize(v_x, 15)) - signed(resize(s_y1, 15));
                s_y1 <= unsigned(signed(s_y1) + resize(shift_right(v_d1, 6), 14));
            end if;

            -- K6 camera gain on the smeared sample: 8 x 10 as two 8 x 5 products, summed in the next stage;
            -- / 16 -> the angle term in 14 bits per turn (max 223 x 1023 / 16 = 14258).  The hold RAM stores it,
            -- so the stored sample is the smear 2 px earlier than before (invisible: the smear is 64 px)
            s_ygl <= s_y1(13 downto 6) * s_cm(4 downto 0);
            s_ygh <= s_y1(13 downto 6) * s_cm(10 downto 5);
            -- (K6 top +25 %: the Osc term can pass a full turn, 223 x 1278 / 16 = 17812; keeping 18 bits drops
            -- whole turns only, and the angle's period is half a turn, so the wrap is seamless.  Video <= 6676.)
            v_g18 := resize(s_ygl, 18) + shift_left(resize(s_ygh, 18), 5);
            s_yg <= v_g18(17 downto 4);
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [D] Tile accumulators.  x: offset in the tile, Spread * column, stepping
    -- per pixel; y: the same per line (ls_* hold the NEXT line's start values).
    ---------------------------------------------------------------------------
    p_acc : process(clk)
        variable v_w   : signed(18 downto 0);
        variable v_b   : signed(27 downto 0);
        variable v_a31 : signed(28 downto 0);
        variable v_sub, v_en : std_logic;
    begin
        if rising_edge(clk) then
            -- line registers: wrap candidates are pre-registered (ls_loc is stable between line starts)
            ls_n1 <= resize(ls_loc + c_yst, 17);
            ls_n2 <= signed(resize(ls_loc, 19)) + c_ymp;
            -- Spread * tile number: - S per step back in the zone's origin searches, + S at each row wrap (one adder)
            v_sub := z_nrun;
            v_en := (z_nrun and z_loc(18)) or (g_start and not ls_n2(18));
            v_b := signed(resize(c_s(22 downto 2), 28));
            if v_sub = '1' then v_b := not v_b; end if;
            v_a31 := (ls_sk & '1') + (v_b & v_sub);
            if s_zone = '1' and (s_zc = 550 or s_zc = 650) then
                ls_sk <= (others => '0');
            elsif v_en = '1' then
                ls_sk <= v_a31(28 downto 1);
            end if;
            if s_zone = '1' and s_zc = 740 then
                ls_loc <= unsigned(z_loc(16 downto 0)); ls_top <= '1';
            elsif g_start = '1' then
                if ls_n2(18) = '0' then
                    ls_loc <= unsigned(ls_n2(16 downto 0)); ls_top <= '0';
                else
                    ls_loc <= ls_n1;
                end if;
            end if;

            -- pixel accumulators (stage 1); the wrap flag of the next step is registered a pixel ahead
            if g_start = '1' then
                a_sk <= c_xsk0; a_loc <= c_xloc0;
                a_wrap <= c_wrap0; a_nw <= '0';
                l_yloc <= ls_loc; l_ysk <= ls_sk; l_top <= ls_top;
            elsif g_avid = '1' then
                if a_wrap = '1' then
                    v_w := signed(resize(a_loc, 19)) + c_xmp;
                    a_loc <= unsigned(v_w(16 downto 0));
                    a_sk  <= a_sk + signed(resize(c_s(22 downto 2), 28));
                    a_nw  <= '1'; a_wrap <= '0';
                else
                    a_loc <= resize(a_loc + c_xst, 17);
                    a_nw  <= '0';
                    if signed(resize(a_loc, 19)) + c_x2mp >= 0 then a_wrap <= '1'; else a_wrap <= '0'; end if;
                end if;
            else
                a_nw <= '0';
            end if;
        end if;
    end process p_acc;

    ---------------------------------------------------------------------------
    -- Fold into the fundamental domain (stages 2 .. 9, quarter virtual px)
    ---------------------------------------------------------------------------
    p_fold : process(clk)
        variable v_a  : signed(11 downto 0);
        variable v_p  : unsigned(19 downto 0);
        variable v_q  : unsigned(18 downto 0);
        variable v_u  : signed(12 downto 0);
    begin
        if rising_edge(clk) then
            s_nw(2) <= a_nw;
            for i in 3 to 18 loop s_nw(i) <= s_nw(i - 1); end loop;

            -- 2: offsets from the tile centre
            q2_tx <= signed(resize(a_loc(16 downto 6), 12)) - signed(resize(c_ph2, 12));
            l_ty  <= signed(resize(l_yloc(16 downto 6), 12)) - signed(resize(c_ph2, 12));
            q2_sk <= a_sk(27 downto 8);
            -- 3: |x|, |y| and the quadrant
            v_a := abs(q2_tx); q3_ax <= unsigned(v_a(10 downto 0)); q3_sxp <= not q2_tx(11);
            v_a := abs(l_ty);  l_ay  <= unsigned(v_a(10 downto 0)); l_syp  <= not l_ty(11);
            q3_sk <= q2_sk;
            -- 4: sum and difference
            q4_s  <= resize(q3_ax, 12) + resize(l_ay, 12);
            q4_dd <= signed(resize(q3_ax, 12)) - signed(resize(l_ay, 12));
            q4_ax <= q3_ax; q4_sxp <= q3_sxp; q4_sk <= q3_sk;
            -- 5: swap about the diagonal; the octagon lattice's small squares lie beyond x + y = P / sqrt 2
            if q4_dd(11) = '0' then q5_hi <= q4_ax; q5_lo <= l_ay; else q5_hi <= l_ay; q5_lo <= q4_ax; end if;
            v_a := abs(q4_dd); q5_df <= unsigned(v_a(10 downto 0));
            q5_s <= q4_s;
            if s_oct = '1' and q4_s > resize(c_pd, 12) then s_sq(5) <= '1'; else s_sq(5) <= '0'; end if;
            for i in 6 to 15 loop s_sq(i) <= s_sq(i - 1); end loop;
            q5_sxp <= q4_sxp; q5_sk <= q4_sk;
            -- 6, 7: (x + y, x - y) * 181 / 256: the reflection about the 22.5 degree line
            q6_sa <= shift_left(resize(q5_s, 20), 7) + shift_left(resize(q5_s, 20), 5);
            q6_sb <= shift_left(resize(q5_s, 20), 4) + shift_left(resize(q5_s, 20), 2) + resize(q5_s, 20);
            q6_da <= shift_left(resize(q5_df, 19), 7) + shift_left(resize(q5_df, 19), 5);
            q6_db <= shift_left(resize(q5_df, 19), 4) + shift_left(resize(q5_df, 19), 2) + resize(q5_df, 19);
            q6_hi <= q5_hi; q6_lo <= q5_lo;
            v_p := q6_sa + q6_sb; q7_xr <= v_p(19 downto 8);
            v_q := q6_da + q6_db; q7_yr <= v_q(18 downto 8);
            q7_hi <= q6_hi; q7_lo <= q6_lo;
            -- 8: Octagon: take the reflected point when it lies nearer the axis
            if s_oct = '1' and q7_yr < q7_lo then
                q8_x <= q7_xr; q8_y <= q7_yr(9 downto 0);
            else
                q8_x <= resize(q7_hi, 12); q8_y <= q7_lo(9 downto 0);
            end if;
            -- 9: distance from the tile edge (either side of it)
            v_u := signed(resize(q8_x, 13)) - signed(resize(c_ph2, 13));
            v_u := abs(v_u);
            q9_u <= unsigned(v_u(9 downto 0));
            q9_y <= q8_y;
        end if;
    end process p_fold;

    ---------------------------------------------------------------------------
    -- Wave engine (stages 6 .. 17): tile index -> theta -> wave ROM -> state
    ---------------------------------------------------------------------------
    p_wave : process(clk)
        variable v_o   : signed(19 downto 0);
        variable v_a   : signed(19 downto 0);
        variable v_l   : signed(19 downto 0);
        variable v_pos : unsigned(11 downto 0);
        variable v_p   : unsigned(17 downto 0);
        variable v_k   : unsigned(7 downto 0);
        variable v_sel : unsigned(13 downto 0);
        variable v_st  : unsigned(13 downto 0);
    begin
        if rising_edge(clk) then
            -- 6: Spread * column and Spread * row; a small square sits half a tile off on both axes
            if s_sq(5) = '0' then v_o := (others => '0'); elsif q5_sxp = '1' then v_o := c_hsp; else v_o := c_hsn; end if;
            q6_ex <= q5_sk + v_o;
            if s_sq(5) = '0' then v_o := (others => '0'); elsif l_syp = '1' then v_o := c_hsp; else v_o := c_hsn; end if;
            q6_ey <= l_ysk(27 downto 8) + v_o;
            -- 7: magnitudes for the radial metric, the linear sum (14 fraction bits)
            v_a := abs(q6_ex); q7_aex <= unsigned(v_a(18 downto 0));
            v_a := abs(q6_ey); q7_aey <= unsigned(v_a(18 downto 0));
            v_l := q6_ex + q6_ey; q7_lin <= unsigned(v_l(13 downto 0));
            -- 8: compare; the phase term
            if q7_aex >= q7_aey then q8_ge <= '1'; else q8_ge <= '0'; end if;
            q8_aex <= q7_aex(14 downto 0); q8_aey <= q7_aey(14 downto 0); q8_lin <= q7_lin;
            q8_nph <= 0 - c_ph(25 downto 12);
            -- 9: octagonal metric max + min / 2 (Radial) or column + row (Linear), + the phase term
            if s_rad = '0' then
                v_sel := q8_lin; q9_xb <= (others => '0');
            elsif q8_ge = '1' then
                v_sel := q8_aex(13 downto 0); q9_xb <= q8_aey(14 downto 1);
            else
                v_sel := q8_aey(13 downto 0); q9_xb <= q8_aex(14 downto 1);
            end if;
            q9_xa <= v_sel + q8_nph;
            -- 10: theta (14 bits per cycle)
            q10_th <= q9_xa + q9_xb;
            -- 11: wave ROM address: quarter-wave fold; Snap = the first sixteenth stretched over the quarter
            v_pos := q10_th(11 downto 0);
            if q10_th(12) = '1' then v_pos := not v_pos; end if;
            if s_snap = '0' then r_w_a <= v_pos(11 downto 2);
            elsif v_pos(11 downto 10) /= "00" then r_w_a <= (others => '1');
            else r_w_a <= v_pos(9 downto 0); end if;
            s_neg(11) <= q10_th(13);
            for i in 12 to 16 loop s_neg(i) <= s_neg(i - 1); end loop;
            -- 12: ROM; 13: fabric copy
            q13_w <= r_w_q(9 downto 0);
            -- 14 .. 16: Depth * wave as gated sums (255 = 1/4 turn)
            q14_p0 <= f_gu(q13_w, c_d(0), 12) + shift_left(f_gu(q13_w, c_d(1), 12), 1);
            q14_p1 <= f_gu(q13_w, c_d(2), 12) + shift_left(f_gu(q13_w, c_d(3), 12), 1);
            q14_p2 <= f_gu(q13_w, c_d(4), 12) + shift_left(f_gu(q13_w, c_d(5), 12), 1);
            q14_p3 <= f_gu(q13_w, c_d(6), 12) + shift_left(f_gu(q13_w, c_d(7), 12), 1);
            q15_qa <= resize(q14_p0, 14) + shift_left(resize(q14_p1, 14), 2);
            q15_qb <= resize(q14_p2, 14) + shift_left(resize(q14_p3, 14), 2);
            v_p := resize(q15_qa, 18) + shift_left(resize(q15_qb, 18), 4);
            q16_aw <= v_p(17 downto 6);

            -- source hold: read address at 13, RAM at 14, fabric at 15; on a sample line the tile's value is
            -- replaced at the tile's last pixel (stage 15), after every read of it on that line.  The top
            -- (partial) row has its own bank, filled on line 0 and read from then on, so line 0 matches its row.
            v_k := k13;
            if s_gav_sr(10) = '1' and s_gav_sr(11) = '0' then v_k := unsigned(c_xk0);
            elsif s_nw(12) = '1' then v_k := k13 + 1; end if;
            k13 <= v_k;
            if s_gav_sr(10) = '1' then sl_ra <= l_top & v_k(6 downto 0); else sl_ra <= l_top & "1000000"; end if;
            k14 <= k13; k15 <= k14;
            q15_src <= unsigned(sl_q(13 downto 0));
            sl_wd <= "00" & std_logic_vector(s_yg);
            sl_wa <= s_wbank & k15(6 downto 0);
            if s_srow = '1' and s_gav_sr(13) = '1' and (s_nw(14) = '1' or s_gav_sr(12) = '0') then sl_we <= '1'; else sl_we <= '0'; end if;

            -- 16: base angle - source term (K6 gain already applied before the hold RAM: at 25 % Video x6,
            -- white = 0.08 turn; Osc x16); the small squares take none
            if s_sq(15) = '1' then v_st := (others => '0');
            else v_st := q15_src; end if;
            q16_st <= c_bang - v_st;
            -- 17: the state: contact angle (14 bits per turn)
            if s_neg(16) = '1' then q17_al <= q16_st + resize(q16_aw, 14);
            else q17_al <= q16_st - resize(q16_aw, 14); end if;
        end if;
    end process p_wave;

    ---------------------------------------------------------------------------
    -- [E] The look: the ray's signed distance, strap coverage, blend table
    ---------------------------------------------------------------------------
    p_look : process(clk)
        variable v_d  : signed(18 downto 0);
        variable v_ab : signed(10 downto 0);
        variable v_c  : signed(11 downto 0);
    begin
        if rising_edge(clk) then
            -- 18: |cos|, |sin| address by quarter turn
            if q17_al(12) = '1' then r_cs_a <= not q17_al(11 downto 3); else r_cs_a <= q17_al(11 downto 3); end if;
            -- 19: ROM; 20: fabric copies, with the folded coordinates from their delay ring
            q20_cos <= r_cs_q(15 downto 8); q20_sin <= r_cs_q(7 downto 0);
            q20_u <= unsigned(vr_q(19 downto 10)); q20_y <= unsigned(vr_q(9 downto 0));
            -- 21 .. 23: Y * |sin| and u * |cos| as gated sums
            q21_a0 <= f_gu(q20_y, q20_sin(0), 12) + shift_left(f_gu(q20_y, q20_sin(1), 12), 1);
            q21_a1 <= f_gu(q20_y, q20_sin(2), 12) + shift_left(f_gu(q20_y, q20_sin(3), 12), 1);
            q21_a2 <= f_gu(q20_y, q20_sin(4), 12) + shift_left(f_gu(q20_y, q20_sin(5), 12), 1);
            q21_a3 <= f_gu(q20_y, q20_sin(6), 12) + shift_left(f_gu(q20_y, q20_sin(7), 12), 1);
            q21_b0 <= f_gu(q20_u, q20_cos(0), 12) + shift_left(f_gu(q20_u, q20_cos(1), 12), 1);
            q21_b1 <= f_gu(q20_u, q20_cos(2), 12) + shift_left(f_gu(q20_u, q20_cos(3), 12), 1);
            q21_b2 <= f_gu(q20_u, q20_cos(4), 12) + shift_left(f_gu(q20_u, q20_cos(5), 12), 1);
            q21_b3 <= f_gu(q20_u, q20_cos(6), 12) + shift_left(f_gu(q20_u, q20_cos(7), 12), 1);
            q22_a01 <= resize(q21_a0, 14) + shift_left(resize(q21_a1, 14), 2);
            q22_a23 <= resize(q21_a2, 14) + shift_left(resize(q21_a3, 14), 2);
            q22_b01 <= resize(q21_b0, 14) + shift_left(resize(q21_b1, 14), 2);
            q22_b23 <= resize(q21_b2, 14) + shift_left(resize(q21_b3, 14), 2);
            q23_pa <= resize(q22_a01, 18) + shift_left(resize(q22_a23, 18), 4);
            q23_pb <= resize(q22_b01, 18) + shift_left(resize(q22_b23, 18), 4);
            -- 24: signed distance to the ray (quarter px)
            v_d := signed(resize(q23_pa, 19)) - signed(resize(q23_pb, 19));
            q24_d <= v_d(18 downto 8);
            -- 25: distance to the strap's edge; which side of the ray
            v_ab := abs(q24_d);
            q25_e <= resize(v_ab, 12) - signed(resize(c_w, 12));
            q25_sd <= not q24_d(10);
            -- 26: coverage in eighths over 2 virtual px (SD: 4 virtual px, about 2 pixels / lines)
            if s_sd = '1' then v_c := shift_right(q25_e, 1) + 4; else v_c := q25_e + 4; end if;
            if v_c < 0 then q26_cov <= (others => '0');
            elsif v_c > 8 then q26_cov <= to_unsigned(8, 4);
            else q26_cov <= unsigned(v_c(3 downto 0)); end if;
            q26_sd <= q25_sd;
            -- 27: table address; 28: RAM; 29: fabric copy, the ground colour outside the generated avid
            tt_ra <= "000" & q26_sd & q26_cov;
            if s_gav_sr(26) = '0' then
                q29_y <= c_gy; q29_c <= c_gc;
            else
                q29_y <= unsigned(tt_q(25 downto 16));
                q29_c(1) <= unsigned(tt_q(15 downto 8)); q29_c(2) <= unsigned(tt_q(7 downto 0));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(26);
        end if;
    end process p_look;

    ---------------------------------------------------------------------------
    -- [F] Output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 (two adjacent
    -- 8-sums of the 8-bit chroma, resynced every line: 10 bits out), luma [1 2 1]/4 delayed to the chroma's
    -- group delay, luma 13/16, blanking gate.  The filters add 4 pixels of
    -- delay: the output register is effective stage 37 = C_LAT + 1.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(11 downto 0);
        variable v_y  : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 30: running 8-sums (the sample of 8 clocks before comes back from its delay ring); they snap
            -- back to 8 x ground once 8 blank samples are in, so a glitched sample can't offset the colours for good
            if s_bah = "00000000" then
                s_bu <= c_gc(1) & "000"; s_bv <= c_gc(2) & "000";
            else
                s_bu <= s_bu + q29_c(1) - unsigned(cr_f(15 downto 8));
                s_bv <= s_bv + q29_c(2) - unsigned(cr_f(7 downto 0));
            end if;
            s_bud <= s_bu; s_bvd <= s_bv;
            s_ysr(0) <= q29_y;
            for i in 1 to 5 loop s_ysr(i) <= s_ysr(i - 1); end loop;
            -- 31: filtered
            v_sc := resize(s_bu, 12) + s_bud; s_u31 <= v_sc(11 downto 2);
            v_sc := resize(s_bv, 12) + s_bvd; s_v31 <= v_sc(11 downto 2);
            v_y := resize(s_ysr(3), 12) + resize(s_ysr(4) & '0', 12) + resize(s_ysr(5), 12);
            s_y31 <= v_y(11 downto 2);
            -- 32: luma 13/16 toward black
            if s_y31 > 64 then v_dl := s_y31 - 64; else v_dl := (others => '0'); end if;
            s_y32 <= s_y31 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u32 <= s_u31; s_v32 <= s_v31;
            -- 33: blanking gate
            if s_gav_sr(C_LAT - 2) = '1' then
                s_y33 <= s_y32; s_u33 <= s_u32; s_v33 <= s_v32;
            else
                s_y33 <= to_unsigned(64, 10); s_u33 <= to_unsigned(512, 10); s_v33 <= to_unsigned(512, 10);
            end if;
        end if;
    end process p_out;

    data_out.y       <= std_logic_vector(s_y33);
    data_out.u       <= std_logic_vector(s_u33);
    data_out.v       <= std_logic_vector(s_v33);
    data_out.hsync_n <= sr_f(2);
    data_out.vsync_n <= sr_f(1);
    data_out.field_n <= sr_f(0);
    data_out.avid    <= s_gav_sr(C_LAT - 1);

end architecture girih;
