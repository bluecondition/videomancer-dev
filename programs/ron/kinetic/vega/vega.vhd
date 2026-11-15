-- Copyright (C) 2026 czartroniks
-- SPDX-License-Identifier: GPL-3.0-only
--
-- Vega v0.1.0 -- Kinetic set.
--
-- A regular grid of inset shapes swelling over a sphere that pushes out of the canvas (Victor Vasarely's Vega
-- series: "Vega-Nor" 1969, "Vega 200"): cells grow at the pole and crowd at the rim.
--
--   element  a grid cell of pitch P; state = the size of its inset shape (fraction of the half cell)
--   lens     u' = u - (u - centre) * m,  m = Bulge * h(r^2 / R^2),  R = 512 virtual px, h from a table of r^2
--            (Bulge: the height of a hemisphere; Pinch: m = -2 Bulge (1 - s)^3); r^2 by second differences;
--            two multiplies per pixel.  The lens centre wanders slowly on its own.
--   cell     coordinates run in "norm px" (virtual px / pitch mantissa) so a cell is 2^n: the cell id and the
--            sub-cell coordinate are the high and low bits of u', v'
--   theta    = Spread * index - phase;  index: Linear = i + j, Radial = max(|i|, |j|) + min / 2
--   size     = 280 + Depth * wave(theta) + source   (of 512 = the half cell), clamped 56 .. 472; the first two
--            terms come from a per-field table by index (the multiply runs serially in the vblank zone)
--   shape    one metric: max(hi, c45 (hi + lo), k * 16-gon): square -> octagon (chamfer) -> disc
--   edges    coverage in sixteenths from (metric - size) over a ramp that follows the lens's local stretch
--            (a table of r^2 and Bulge, quarter-octave steps)
--   colour   a per-field table: ground + (C - ground) * coverage / 16, C = the cell's colour: A, or with Grade
--            the scale A -> B -> accent stepping with the index
--
--   K1  Scale    grid pitch 24 .. 255 virtual lines (exponential, glided)
--   K2  Shape    square .. octagon .. disc (24 steps)
--   K3  Depth    size swing (square law)
--   K4  Rate     bipolar, centre = still; 60 s per cycle .. 2 cycles per second
--   K5  Colour   the 16 shared palettes   [K6 camera pass: was K6; Spread fixed at 274]
--   K6  Camera   camera (and Osc) strength: size term = smeared luma * 1.5 g (Osc * 2 g), g = K6 / 512 up
--                to 50 %, then 3/8 of that slope (glided): 0 = none (= a black input), 0.5 at 25 %, 1 = the
--                v0.1.0 gain at the 50 % default (already near the size cap), 1.37 at 100 %
--   S7  Wave     Smooth / Snap
--   S8  Field    Linear / Radial
--   S9  Lens     Bulge / Pinch
--   S10 Grade    Off / On
--   S11 Source   Video / Osc
--   P12 BULGE    0 % a flat grid .. 100 % a globe filling the screen height (glided)
--
-- Chassis sections lifted from louver.vhd (Kinetic pilot):
--   [A] timing ID -> virtual-1080 steps, vblank zone, width, knob glides, per-field sequencer (serial multiplier)
--   [B] generated avid
--   [C] source field: Osc lattice, luma smear, per-cell hold
--   [D] wave engine: per-field wave table by cell index
--   [E] the look: lens, cells, shape metric, coverage, blend table
--   [F] output conditioning: luma [1 2 1], chroma [1 2 2 2 2 2 2 2 1] with resync, luma 13/16, blanking gate
-- One latency (C_LAT = 33) in every mode.  Colours in the hardware convention (u = Cr, v = Cb).

-- K6 camera pass 2026-11-15: K6 = Camera (camera strength, new); K5 = Colour (was K6); K5 Spread fixed at 274 (C_KFIX_5).
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.video_timing_pkg.all;
use work.video_stream_pkg.all;
use work.core_pkg.all;
use work.vega_pkg.all;
use work.all;

architecture vega of program_top is
    constant C_KFIX_5 : std_logic_vector(9 downto 0) := std_logic_vector(to_unsigned(274, 10));   -- K6 camera pass: fixed knob
    constant C_LAT      : integer := 33;      -- = processing_delay_clks
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
    -- field lines from the top to the screen centre (540 virtual lines)
    function f_half(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(121, 11);
            when 4          => return to_unsigned(240, 11);
            when 8          => return to_unsigned(144, 11);
            when 12         => return to_unsigned(288, 11);
            when 5 | 6 | 14 => return to_unsigned(360, 11);
            when 1 | 2 | 10 => return to_unsigned(270, 11);
            when others     => return to_unsigned(540, 11);
        end case;
    end function;
    -- Osc lattice y step = y step * 121 / 256
    function f_oys(t : std_logic_vector(3 downto 0)) return unsigned is
    begin
        case to_integer(unsigned(t)) is
            when 0          => return to_unsigned(537, 10);
            when 4          => return to_unsigned(272, 10);
            when 8          => return to_unsigned(453, 10);
            when 12         => return to_unsigned(226, 10);
            when 5 | 6 | 14 => return to_unsigned(181, 10);
            when 1 | 2 | 10 => return to_unsigned(242, 10);
            when others     => return to_unsigned(121, 10);
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
    function f_max(a, b : unsigned) return unsigned is
    begin
        if a >= b then return a; else return b; end if;
    end function;

    attribute no_rw_check : boolean;

    -- ROMs (one read port each, registered address and data)
    signal r_ex_a : unsigned(7 downto 0) := (others => '0');
    signal r_ex_q : unsigned(15 downto 0) := (others => '0');
    signal r_w_a  : unsigned(8 downto 0) := (others => '0');
    signal r_w_q  : unsigned(11 downto 0) := (others => '0');
    signal r_h_a  : unsigned(10 downto 0) := (others => '0');
    signal r_h_q  : unsigned(11 downto 0) := (others => '0');
    signal r_l_a  : unsigned(9 downto 0) := (others => '0');
    signal r_l_q  : unsigned(7 downto 0) := (others => '0');
    signal r_pl_a : unsigned(7 downto 0) := (others => '0');
    signal r_pl_q : std_logic_vector(31 downto 0) := (others => '0');

    type t_r16 is array (0 to 255) of std_logic_vector(15 downto 0);
    -- [D] wave table, refilled every field: by signed half-cell index -> {colour step 4, 00, 280 +- Depth * wave (10)}
    signal wt : t_r16 := (others => (others => '0'));
    attribute no_rw_check of wt : signal is true;
    signal wt_ra, wt_wa : unsigned(7 downto 0) := (others => '0');
    signal wt_q, wt_wd  : std_logic_vector(15 downto 0) := (others => '0');
    signal wt_we : std_logic := '0';
    -- [C] source hold per cell: address = row parity & column, word = {field parity & row(5..1), source term 10}
    signal sh : t_r16 := (others => (others => '0'));
    attribute no_rw_check of sh : signal is true;
    signal sh_ra, sh_wa : unsigned(7 downto 0) := (others => '0');
    signal sh_q, sh_wd  : std_logic_vector(15 downto 0) := (others => '0');
    signal sh_we : std_logic := '0';
    -- delay rings (write at rc + d, read at rc, RAM register, fabric register: d + 2 stages)
    signal ur : t_r16 := (others => (others => '0'));          -- u (norm px Q3): stage 2 -> 11
    attribute no_rw_check of ur : signal is true;
    signal ur_q : std_logic_vector(15 downto 0) := (others => '0');
    signal dr : t_r16 := (others => (others => '0'));          -- u - lens centre (Q2): stage 2 -> 7
    attribute no_rw_check of dr : signal is true;
    signal dr_q : std_logic_vector(15 downto 0) := (others => '0');
    signal lr : t_r16 := (others => (others => '0'));          -- ramp exponent: stage 5 -> 20
    attribute no_rw_check of lr : signal is true;
    signal lr_q, lr_f : std_logic_vector(15 downto 0) := (others => '0');
    signal gr : t_r16 := (others => (others => '0'));          -- colour step: stage 16 -> 23
    attribute no_rw_check of gr : signal is true;
    signal gr_q, gr_f : std_logic_vector(15 downto 0) := (others => '0');
    signal cr : t_r16 := (others => (others => '0'));          -- chroma 8 back for the running 8-sums
    attribute no_rw_check of cr : signal is true;
    signal cr_q, cr_f : std_logic_vector(15 downto 0) := (others => '0');
    -- [E] blend table, refilled every field: T(colour step, coverage) = {Y 10, U 8, V 8}
    type t_tt is array (0 to 255) of std_logic_vector(31 downto 0);
    signal tt : t_tt := (others => (others => '0'));
    attribute no_rw_check of tt : signal is true;
    signal tt_ra, tt_wa : unsigned(7 downto 0) := (others => '0');
    signal tt_q, tt_wd  : std_logic_vector(31 downto 0) := (others => '0');
    signal tt_we : std_logic := '0';
    signal t_run, t_row : std_logic := '0';
    signal t_g   : unsigned(3 downto 0) := (others => '0');
    signal t_c   : unsigned(3 downto 0) := (others => '0');
    signal t_cy  : signed(15 downto 0) := (others => '0');      -- 16 x the row's colour
    signal t_sy  : signed(15 downto 0) := (others => '0');
    signal t_ey  : signed(19 downto 0) := (others => '0');
    type t_s14a is array (1 to 2) of signed(13 downto 0);
    type t_s18a is array (1 to 2) of signed(17 downto 0);
    signal t_cc, t_sc : t_s14a := (others => (others => '0'));
    signal t_ec : t_s18a := (others => (others => '0'));
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
    signal s_idle : unsigned(9 downto 0) := (others => '0');
    signal s_zone, s_zone_d, s_zgo : std_logic := '0';
    signal s_zc   : unsigned(10 downto 0) := (others => '0');
    signal s_avq  : std_logic := '0';
    signal s_hcnt, s_wmax : unsigned(11 downto 0) := (others => '0');
    signal s_w    : unsigned(11 downto 0) := to_unsigned(1920, 12);
    signal s_ok   : std_logic := '0';

    -- knobs: glided (Q3), raw rate, switches, palette
    signal s_g1 : unsigned(12 downto 0) := to_unsigned(450 * 8, 13);
    signal s_gp : unsigned(12 downto 0) := to_unsigned(640 * 8, 13);
    signal s_gk : unsigned(12 downto 0) := to_unsigned(512 * 8, 13);   -- K6 camera pass: the Camera glide
    signal c_cam : unsigned(6 downto 0) := to_unsigned(48, 7);         -- camera coefficient, Q5 (48 = x1.5)
    signal s_ke  : unsigned(9 downto 0) := to_unsigned(512, 10);       -- the camera law's knee (K6 camera pass)
    signal s_gm : unsigned(12 downto 0) := (others => '0');
    signal s_gd : signed(14 downto 0) := (others => '0');
    signal s_gr : unsigned(9 downto 0) := (others => '0');
    signal s_k4 : unsigned(9 downto 0) := to_unsigned(640, 10);
    signal s_vm : unsigned(8 downto 0) := (others => '0');
    signal s_vneg : std_logic := '0';
    signal s_snap, s_rad, s_pin, s_grade, s_src : std_logic := '0';
    signal s_pal : unsigned(3 downto 0) := to_unsigned(3, 4);
    signal z_gp  : std_logic_vector(1 downto 0) := "00";
    signal z_gc  : unsigned(2 downto 0) := (others => '0');
    signal z_grun : std_logic := '0';

    -- field serial multiplier (24 clocks per slot), serial shifter, sequencer scratch
    signal f_a, f_acc : unsigned(37 downto 0) := (others => '0');
    signal z_fa : unsigned(19 downto 0) := (others => '0');
    signal f_b, z_fb : unsigned(20 downto 0) := (others => '0');
    signal z_sel : unsigned(3 downto 0) := (others => '0');
    signal z_pre, z_ld : std_logic := '0';
    signal z_e   : unsigned(10 downto 0) := (others => '0');
    signal z_sh  : unsigned(2 downto 0) := (others => '0');
    signal z_nz  : std_logic := '0';
    signal z_lq  : unsigned(1 downto 0) := (others => '0');
    signal z_shn : unsigned(2 downto 0) := (others => '0');
    signal z_mt  : unsigned(15 downto 0) := to_unsigned(32768, 16);
    signal z_sp  : unsigned(4 downto 0) := (others => '0');
    signal s_wx  : unsigned(15 downto 0) := (others => '0');            -- lens wander phases
    signal s_wy  : unsigned(15 downto 0) := to_unsigned(16384, 16);

    -- per-field constants
    signal c_sh   : unsigned(1 downto 0) := "10";                       -- cell = 16 * 2^sh norm px
    signal c_invm : unsigned(16 downto 0) := to_unsigned(65536, 17);    -- 1 / pitch mantissa, Q16
    signal c_sx   : unsigned(18 downto 0) := to_unsigned(65536, 19);    -- norm px per pixel, Q16
    signal c_sy   : unsigned(19 downto 0) := to_unsigned(65536, 20);    -- norm px per line, Q16
    signal c_hc   : signed(27 downto 0);                                -- half a cell, Q16
    signal c_un0  : signed(14 downto 0) := (others => '0');             -- u at pixel 0, Q3
    signal c_cuq, c_cvq : signed(12 downto 0) := (others => '0');       -- lens centre, Q2
    signal c_cxm  : unsigned(8 downto 0) := (others => '0');            -- lens centre, half virtual px (magnitude)
    signal c_cym  : unsigned(5 downto 0) := (others => '0');
    signal c_cxn, c_cyn : std_logic := '0';
    signal c_mx   : unsigned(12 downto 0) := (others => '0');           -- lens centre - pixel 0, half px
    signal c_my   : unsigned(15 downto 0) := (others => '0');           -- lens centre - line 0, 1/16 half px
    signal c_qx0  : unsigned(23 downto 0) := (others => '0');
    signal c_ix0  : signed(17 downto 0) := (others => '0');
    signal c_ixx  : unsigned(5 downto 0) := to_unsigned(8, 6);
    signal c_iyy  : unsigned(17 downto 0) := (others => '0');
    signal c_b8   : unsigned(7 downto 0) := (others => '0');            -- bulge amount
    signal c_d    : unsigned(7 downto 0) := (others => '0');            -- depth
    signal c_s    : unsigned(22 downto 0) := (others => '0');           -- spread per cell, Q24 cycles
    signal c_j4   : unsigned(3 downto 0) := (others => '0');            -- chamfer coefficient - 1/2, Q6
    signal c_t4   : unsigned(3 downto 0) := (others => '0');            -- 16-gon shrink, Q7
    signal c_rnd  : std_logic := '0';
    signal c_lf   : unsigned(3 downto 0) := to_unsigned(4, 4);          -- flat-grid ramp exponent, quarter octaves
    signal c_ph   : unsigned(25 downto 0) := (others => '0');           -- wave phase, Q26 cycles
    signal c_dph  : unsigned(21 downto 0) := (others => '0');
    type t_u10 is array (natural range <>) of unsigned(9 downto 0);
    type t_u8  is array (1 to 2) of unsigned(7 downto 0);
    -- palette: luma 10 bits, chroma 8 bits
    signal c_gy   : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal c_ay, c_by, c_ky : unsigned(9 downto 0) := to_unsigned(64, 10);
    signal c_gc   : t_u8 := (others => to_unsigned(128, 8));
    signal c_ac, c_bc, c_kc : t_u8 := (others => to_unsigned(128, 8));

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
    signal s_sav, s_savd : std_logic := '0';
    signal s_ext : unsigned(5 downto 0) := (others => '0');
    signal s_y1  : unsigned(13 downto 0) := (others => '0');
    signal s_st  : unsigned(9 downto 0) := (others => '0');
    signal s_cpa : unsigned(11 downto 0) := (others => '0');   -- K6 camera pass: smear * c_cam partial sums
    signal s_cpb : unsigned(10 downto 0) := (others => '0');

    -- [D] wave table fill (12 clocks per entry: ROM, then Depth * wave by shift-adds)
    signal w_acc : unsigned(23 downto 0) := (others => '0');
    signal w_j   : unsigned(7 downto 0) := (others => '0');
    signal w_run, w_neg : std_logic := '0';
    signal w_ph  : unsigned(3 downto 0) := (others => '0');
    signal w_mc  : unsigned(9 downto 0) := (others => '0');
    signal w_mb  : unsigned(7 downto 0) := (others => '0');

    -- [E] line registers (the NEXT line's start values), this line's values, pixel accumulators (stage 1)
    signal ls_vn : signed(27 downto 0) := (others => '0');
    signal ls_qy : signed(30 downto 0) := (others => '0');
    signal ls_iy : signed(26 downto 0) := (others => '0');
    signal l_vnq : signed(14 downto 0) := (others => '0');
    signal l_qy8 : unsigned(20 downto 0) := (others => '0');
    signal l_dy  : signed(12 downto 0) := (others => '0');
    signal a_un  : signed(27 downto 0) := (others => '0');
    signal a_qx  : signed(24 downto 0) := (others => '0');
    signal a_ix  : signed(17 downto 0) := (others => '0');
    -- lens
    signal q2_un : signed(14 downto 0) := (others => '0');
    signal q2_dx : signed(12 downto 0) := (others => '0');
    signal q2_in, q3_in : std_logic := '0';
    signal q4_h  : unsigned(9 downto 0) := (others => '0');
    signal q4_l  : signed(4 downto 0) := (others => '0');
    signal q5_p0, q5_p1, q5_p2, q5_p3 : unsigned(11 downto 0) := (others => '0');
    signal q5_l4 : signed(4 downto 0) := (others => '0');
    signal q6_qa, q6_qb : unsigned(13 downto 0) := (others => '0');
    signal q7_m  : unsigned(9 downto 0) := (others => '0');
    signal q7_dx : signed(12 downto 0) := (others => '0');
    type t_s15a is array (0 to 4) of signed(14 downto 0);
    signal q8_px, q8_py : t_s15a := (others => (others => '0'));
    signal q9_ax, q9_bx, q9_ay, q9_by : signed(16 downto 0) := (others => '0');
    signal q9_cx, q9_cy, q10_ux, q10_uy : signed(14 downto 0) := (others => '0');
    signal q10_tx, q10_ty : signed(20 downto 0) := (others => '0');
    signal q11_du, q11_dv : signed(13 downto 0) := (others => '0');
    signal q11_un : signed(14 downto 0) := (others => '0');
    signal q12_u, q12_v : signed(14 downto 0) := (others => '0');
    -- cells
    signal q13_ci, q13_cj : signed(7 downto 0) := (others => '0');
    signal q13_a, q13_b, q14_hi, q14_lo : unsigned(9 downto 0) := (others => '0');
    signal q14_id, q15_id, q16_id : std_logic_vector(12 downto 0) := (others => '0');   -- row(5..0) & column(6..0)
    signal q15_new, q16_new : std_logic := '0';
    signal q16_hq : std_logic_vector(15 downto 0) := (others => '0');
    signal q16_fb, q17_fb, q17_st : unsigned(9 downto 0) := (others => '0');
    signal q16_gi : unsigned(3 downto 0) := (others => '0');
    signal q18_s, q19_s : unsigned(9 downto 0) := (others => '0');
    -- shape metric
    signal q15_sm, q16_sm : unsigned(10 downto 0) := (others => '0');
    signal q15_hi, q16_hi, q17_hi : unsigned(9 downto 0) := (others => '0');
    signal q15_x22, q15_y22 : unsigned(9 downto 0) := (others => '0');
    signal q16_ja, q16_jb : unsigned(12 downto 0) := (others => '0');
    signal q16_d22, q17_mr, q17_d45, q18_mr, q18_m8, q19_d : unsigned(10 downto 0) := (others => '0');
    signal q18_pt : unsigned(14 downto 0) := (others => '0');
    -- coverage, colour
    signal q20_e  : signed(8 downto 0) := (others => '0');
    signal q21_v  : signed(14 downto 0) := (others => '0');
    signal q21_ex : signed(2 downto 0) := (others => '0');
    signal q22_dl : signed(5 downto 0) := (others => '0');
    signal q23_cov : unsigned(3 downto 0) := (others => '0');
    signal q29_y  : unsigned(9 downto 0) := to_unsigned(64, 10);        -- the pixel (stage 26)
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

    p_hc : process(c_sh)
    begin
        c_hc <= (others => '0');
        c_hc(19 + to_integer(c_sh)) <= '1';
    end process p_hc;

    ---------------------------------------------------------------------------
    -- Block RAM: ROMs, tables, hold, delay rings (one write statement each;
    -- none ever uses data read from the address it is writing)
    ---------------------------------------------------------------------------
    p_rom : process(clk)
    begin
        if rising_edge(clk) then
            r_ex_q <= C_VG_EXP2(to_integer(r_ex_a));
            r_w_q  <= C_VG_SIN(to_integer(r_w_a));
            r_h_q  <= C_VG_LENS(to_integer(r_h_a));
            r_l_q  <= C_VG_AA(to_integer(r_l_a));
            r_pl_q <= C_VG_PAL(to_integer(r_pl_a));
        end if;
    end process p_rom;

    p_wt : process(clk)
    begin
        if rising_edge(clk) then
            wt_q <= wt(to_integer(wt_ra));
            if wt_we = '1' then wt(to_integer(wt_wa)) <= wt_wd; end if;
        end if;
    end process p_wt;

    p_sh : process(clk)
    begin
        if rising_edge(clk) then
            sh_q <= sh(to_integer(sh_ra));
            if sh_we = '1' then sh(to_integer(sh_wa)) <= sh_wd; end if;
        end if;
    end process p_sh;

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
            ur(to_integer(rc + 7)) <= '0' & std_logic_vector(q2_un);
            ur_q <= ur(to_integer(rc));
            dr(to_integer(rc + 3)) <= "000" & std_logic_vector(q2_dx);
            dr_q <= dr(to_integer(rc));
            lr(to_integer(rc + 13)) <= "00000000000" & std_logic_vector(q5_l4);
            lr_q <= lr(to_integer(rc));
            lr_f <= lr_q;
            gr(to_integer(rc + 5)) <= "000000000000" & std_logic_vector(q16_gi);
            gr_q <= gr(to_integer(rc));
            gr_f <= gr_q;
            cr(to_integer(rc + 6)) <= std_logic_vector(q29_c(1)) & std_logic_vector(q29_c(2));
            cr_q <= cr(to_integer(rc));
            cr_f <= cr_q;
            sr(to_integer(rc + (C_LAT - 2))) <= '1' & data_in.hsync_n & data_in.vsync_n & data_in.field_n;
            sr_q <= sr(to_integer(rc));
            sr_f <= sr_q;
            rc <= rc + 1;
        end if;
    end process p_ring;

    ---------------------------------------------------------------------------
    -- [A] Controls, vblank zone, glides, per-field sequencer (one small
    -- operation per zone clock; quasi-static but timed at the pixel clock)
    ---------------------------------------------------------------------------
    p_ctrl : process(clk)
        variable v_s  : signed(10 downto 0);
        variable v_m  : unsigned(9 downto 0);
        variable v_d  : signed(10 downto 0);
        variable v_t  : unsigned(28 downto 0);
        variable v_b  : unsigned(25 downto 0);
        variable v_a27 : unsigned(26 downto 0);
        variable v_k24 : unsigned(14 downto 0);
        variable v_wh : unsigned(10 downto 0);
        variable v_st : unsigned(8 downto 0);
        variable v_cn : signed(27 downto 0);
        variable v_xx : unsigned(4 downto 0);
        variable v_un : signed(27 downto 0);
    begin
        if rising_edge(clk) then
            s_tid <= registers_in(8)(3 downto 0);
            s_ys  <= f_vstep_y(s_tid);
            s_xsr <= f_vstep_x(s_tid);
            v_wh := s_w(11 downto 1);
            v_st := s_ys(11 downto 3);

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
                s_snap  <= registers_in(6)(0);
                s_rad   <= registers_in(6)(1);
                s_pin   <= registers_in(6)(2);
                s_grade <= registers_in(6)(3);
                s_src   <= registers_in(6)(4);
                s_xs    <= s_xsr;
                case to_integer(s_xsr) is
                    when 4      => s_oxs <= to_unsigned(204, 9);
                    when 3      => s_oxs <= to_unsigned(153, 9);
                    when others => s_oxs <= to_unsigned(102, 9);
                end case;
                -- Colour: 16 palettes with hysteresis
                v_d := signed(resize(unsigned(registers_in(4)), 11)) - signed(resize(s_pal & "100000", 11));
                if v_d > 40 or v_d < -40 then s_pal <= unsigned(registers_in(4)(9 downto 6)); end if;
                -- lens wander
                s_wx <= s_wx + 38;
                s_wy <= s_wy + 27;
            end if;
            g_wm1 <= s_w - 1;
            s_oys <= f_oys(s_tid);

            -- shared glide for Scale and Bulge (1/8 of the gap per field, +-2 deadband): four clocks per channel
            if s_zgo = '1' then
                z_gp <= "00"; z_gc <= "000"; z_grun <= '1';
            elsif z_grun = '1' then
                case z_gp is
                    when "00" =>
                        if z_gc(1) = '1' then s_gm <= s_gk; s_gr <= unsigned(registers_in(5));   -- K6 Camera
                        elsif z_gc(0) = '0' then s_gm <= s_g1; s_gr <= unsigned(registers_in(0));
                        else s_gm <= s_gp; s_gr <= unsigned(registers_in(7)); end if;
                        z_gp <= "11";
                    when "11" =>
                        s_gd <= signed(resize(s_gr & "000", 15)) - signed(resize(s_gm, 15));
                        z_gp <= "01";
                    when "01" =>                           -- deadband +-2 codes (the camera, channel 2: +-4)
                        if C_SIM_SNAP then
                            null;
                        elsif (s_gd > 16 or s_gd < -16) and (z_gc(1) = '0' or s_gd > 32 or s_gd < -32) then
                            s_gd <= shift_right(s_gd, 3);
                        else
                            s_gd <= (others => '0');
                        end if;
                        z_gp <= "10";
                    when others =>
                        if z_gc(1) = '1' then s_gk <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        elsif z_gc(0) = '0' then s_g1 <= unsigned(signed(s_gm) + s_gd(12 downto 0));
                        else s_gp <= unsigned(signed(s_gm) + s_gd(12 downto 0)); end if;
                        z_gp <= "00";
                        if z_gc = 2 then z_grun <= '0'; end if;
                        z_gc <= z_gc + 1;
                end case;
            end if;

            -- serial products: slot k loads at zc = 32 + 32 k, 24 iterations, stored at 60 + 32 k.  The slot
            -- number is decoded at 30 + 32 k and the operands pre-muxed at 31 + 32 k (no zc decode on the
            -- multiplier's path).
            z_pre <= '0';
            if s_zone = '1' and s_zc(4 downto 0) = 30 and s_zc(10 downto 9) = "00" and s_zc(8 downto 5) /= 15 then
                z_sel <= s_zc(8 downto 5); z_pre <= '1';
            end if;
            z_ld <= z_pre;
            if z_pre = '1' then
                z_fa <= (others => '0'); z_fb <= (others => '0');
                case to_integer(z_sel) is
                    when 0  => z_fa(12 downto 0) <= s_g1;                 z_fb <= to_unsigned(6991, 21);      -- pitch exponent
                    when 1  =>                                                                                -- depth^2
                        z_fa(12 downto 3) <= unsigned(registers_in(2)); z_fb(12 downto 3) <= unsigned(registers_in(2));
                    when 2  => z_fa(16 downto 0) <= c_invm;               z_fb <= resize(s_ys, 21);           -- y step
                    when 3  => z_fa(16 downto 0) <= c_invm;               z_fb <= resize(c_cxm, 21);          -- lens centre x
                    when 4  => z_fa(16 downto 0) <= c_invm;               z_fb <= resize(c_cym, 21);          -- lens centre y
                    when 5  => z_fa(18 downto 0) <= c_sx;                 z_fb <= resize(v_wh, 21);           -- u at pixel 0
                    when 6  => z_fa(19 downto 0) <= c_sy;                 z_fb <= resize(f_half(s_tid), 21);  -- v at line 0
                    when 7  => z_fa(12 downto 0) <= c_mx;                 z_fb <= resize(c_mx, 21);           -- dx^2 at pixel 0
                    when 8  => z_fa(10 downto 0) <= f_half(s_tid);        z_fb <= resize(v_st, 21);           -- lines to the centre
                    when 9  => z_fa(15 downto 0) <= c_my;                 z_fb <= resize(c_my, 21);           -- dy^2 at line 0
                    when 10 => z_fa(15 downto 0) <= c_my;                 z_fb <= resize(v_st, 21);
                    when 11 => z_fa(8 downto 0)  <= v_st;                 z_fb <= resize(v_st, 21);
                    when 12 => z_fa(8 downto 0)  <= s_vm;                 z_fb <= to_unsigned(900, 21);       -- rate exponent
                    when 13 =>                                                                                -- spread^2
                        z_fa(12 downto 3) <= unsigned(C_KFIX_5); z_fb(12 downto 3) <= unsigned(C_KFIX_5);
                    when others => z_fa(15 downto 0) <= z_mt;             z_fb <= to_unsigned(1193046, 21);   -- rate (14)
                end case;
            end if;
            if z_ld = '1' then
                f_acc <= (others => '0');
                f_a <= resize(z_fa, 38); f_b <= z_fb;
            else
                if f_b(0) = '1' then f_acc <= f_acc + f_a; end if;
                f_a <= f_a(36 downto 0) & '0';
                f_b <= '0' & f_b(20 downto 1);
            end if;

            -- serial shifter for the rate's 2^n scaling (no barrel)
            if z_shn /= 0 then
                z_shn <= z_shn - 1;
                c_dph <= c_dph(20 downto 0) & '0';
            end if;

            if s_zone = '1' then
                case to_integer(s_zc) is
                    -- lens centre from the wave ROM (addresses issued by p_wfill at 4 and 7)
                    when 7  => c_cxm <= r_w_q(9 downto 1); c_cxn <= s_wx(15);
                    when 10 => c_cym <= r_w_q(9 downto 4); c_cyn <= s_wy(15);
                    -- lens centre - pixel 0 in half virtual px; the r^2 increments
                    when 12 =>
                        case to_integer(s_xs) is
                            when 4      => c_mx <= v_wh & "00";
                            when 3      => c_mx <= resize(v_wh & '0', 13) + resize(v_wh, 13);
                            when others => c_mx <= resize(v_wh & '0', 13);
                        end case;
                    when 13 =>
                        if c_cxn = '1' then c_mx <= c_mx - resize(c_cxm, 13); else c_mx <= c_mx + resize(c_cxm, 13); end if;
                    when 14 =>
                        case to_integer(s_xs) is
                            when 4      => c_ix0 <= to_signed(16, 18) - signed(resize(c_mx & "000", 18)); c_ixx <= to_unsigned(32, 6);
                            when 3      => c_ix0 <= to_signed(9, 18) - signed(resize(c_mx & "00", 18));   c_ixx <= to_unsigned(18, 6);
                            when others => c_ix0 <= to_signed(4, 18) - signed(resize(c_mx & "00", 18));   c_ixx <= to_unsigned(8, 6);
                        end case;
                    when 15 =>
                        if s_xs = 3 then c_ix0 <= c_ix0 - signed(resize(c_mx & '0', 18)); end if;
                    -- shape: 24 steps; bulge amount
                    when 16 =>
                        v_k24 := resize(unsigned(registers_in(1)) & "0000", 15) + resize(unsigned(registers_in(1)) & "000", 15);
                        z_sp <= v_k24(14 downto 10);
                    when 17 =>
                        if z_sp <= 13 then
                            c_j4 <= z_sp(3 downto 0); c_t4 <= (others => '0'); c_rnd <= '0';
                        else
                            c_j4 <= to_unsigned(13, 4); c_t4 <= resize(23 - z_sp, 4); c_rnd <= '1';
                        end if;
                        c_b8 <= s_gp(12 downto 5);
                    -- palette: ground, A, B, accent (chroma kept to 8 bits)
                    -- K6 camera pass: camera coefficient (Q5): Video 1.5 g, Osc 2 g; g = ke / 512 with ke = K6 up
                    -- to 50 % (g 1 = v0.1.0), then 512 + 3 (K6 - 512) / 8 (g 1.37 at 100 %: the size cap)
                    when 40 =>
                        if s_gk(12) = '0' then s_ke <= s_gk(12 downto 3);
                        else s_ke <= "10" & (resize(s_gk(11 downto 5), 8) + resize(s_gk(11 downto 6), 8)); end if;
                    when 41 =>
                        if s_src = '1' then v_t := resize(s_ke & "00", 29);
                        else v_t := resize(s_ke & '0', 29) + resize(s_ke, 29); end if;
                        c_cam <= v_t(11 downto 5);
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
                        c_ky <= unsigned(r_pl_q(31 downto 22)); c_kc(1) <= unsigned(r_pl_q(21 downto 14)); c_kc(2) <= unsigned(r_pl_q(11 downto 4));

                    -- pitch: E = 150 + k1 * 873 / 1023 (Q8 octaves), P = 16 * 2^E; cell = 16 * 2^sh norm px
                    when 60 => z_e <= f_acc(26 downto 16) + 150;
                    when 61 =>
                        r_ex_a <= 0 - z_e(7 downto 0); c_sh <= z_e(9 downto 8); z_lq <= z_e(7 downto 6);
                        if z_e(7 downto 0) = 0 then z_nz <= '0'; else z_nz <= '1'; end if;
                    when 63 =>
                        if z_nz = '1' then c_invm <= '0' & r_ex_q; else c_invm <= to_unsigned(65536, 17); end if;
                    when 64 =>
                        case to_integer(s_xs) is
                            when 4      => c_sx <= resize(c_invm & '0', 19); c_lf <= resize(z_lq, 4);
                            when 3      => c_sx <= resize(c_invm, 19) + resize(c_invm(16 downto 1), 19); c_lf <= resize(z_lq, 4) + 2;
                            when others => c_sx <= resize(c_invm, 19); c_lf <= resize(z_lq, 4) + 4;
                        end case;
                    when 92 =>                       -- depth = (k3 / 1023)^2 * 223 / 1024 of the half cell
                        v_t := shift_left(resize(f_acc(25 downto 0), 29), 3) - resize(f_acc(25 downto 0), 29);
                        c_d <= v_t(28 downto 21);
                    when 124 => c_sy <= f_acc(27 downto 8);
                    when 156 =>
                        v_cn := signed(resize(f_acc(25 downto 1), 28));
                        if c_cxn = '1' then v_cn := c_hc - v_cn; else v_cn := c_hc + v_cn; end if;
                        c_cuq <= v_cn(26 downto 14);
                    when 188 =>
                        v_cn := signed(resize(f_acc(25 downto 1), 28));
                        if c_cyn = '1' then v_cn := c_hc - v_cn; else v_cn := c_hc + v_cn; end if;
                        c_cvq <= v_cn(26 downto 14);
                    when 220 =>
                        v_un := c_hc - signed(resize(f_acc(26 downto 0), 28));
                        c_un0 <= v_un(27 downto 13);
                    when 284 => c_qx0 <= f_acc(23 downto 0);
                    when 316 =>
                        if c_cyn = '1' then c_my <= f_acc(15 downto 0) - resize(c_cym & "0000", 16);
                        else c_my <= f_acc(15 downto 0) + resize(c_cym & "0000", 16); end if;
                    when 412 => c_iyy <= f_acc(16 downto 0) & '0';
                    -- rate: 2^(m * 6.9 / 503) / 3600 cycles per field
                    when 444 => z_e <= f_acc(18 downto 8);
                    when 445 => r_ex_a <= z_e(7 downto 0); z_sh <= z_e(10 downto 8);
                    when 447 => z_mt <= r_ex_q;
                    when 476 => c_s <= f_acc(25 downto 3);
                    when 508 =>
                        if s_vm = 0 then c_dph <= (others => '0'); else c_dph <= resize(f_acc(36 downto 21), 22); end if;
                        z_shn <= z_sh;
                    -- wave phase: one adder, add or subtract
                    when 544 =>
                        v_b := resize(c_dph, 26);
                        if s_vneg = '1' then v_b := not v_b; end if;
                        v_a27 := (c_ph & '1') + (v_b & s_vneg);
                        c_ph <= v_a27(26 downto 1);
                    when others => null;
                end case;
            end if;

            s_gav_sr(0) <= g_avid;
            for i in 1 to C_LAT - 1 loop
                s_gav_sr(i) <= s_gav_sr(i - 1);
            end loop;
        end if;
    end process p_ctrl;

    ---------------------------------------------------------------------------
    -- Blend table fill (zone, zc 40 .. ~315; adders only).  Row g = the cell
    -- colour C_g (0 .. 7: A -> B, 8 .. 15: B -> accent); column c = coverage:
    -- T = ground + (C_g - ground) * c / 16 (c = 15 holds C_g itself).
    ---------------------------------------------------------------------------
    p_fill : process(clk)
    begin
        if rising_edge(clk) then
            tt_we <= '0';
            if s_zone = '1' and s_zc = 40 then
                t_run <= '1'; t_row <= '1'; t_g <= (others => '0');
                t_cy <= signed(resize(c_ay & "0000", 16));
                for i in 1 to 2 loop t_cc(i) <= signed(resize(c_ac(i) & "0000", 14)); end loop;
            elsif t_run = '1' then
                if t_row = '1' then
                    t_sy <= t_cy - signed(resize(c_gy & "0000", 16));
                    t_ey <= signed(resize(c_gy & "00000000", 20));
                    for i in 1 to 2 loop
                        t_sc(i) <= t_cc(i) - signed(resize(c_gc(i) & "0000", 14));
                        t_ec(i) <= signed(resize(c_gc(i) & "00000000", 18));
                    end loop;
                    t_c <= (others => '0'); t_row <= '0';
                else
                    tt_we <= '1'; tt_wa <= t_g & t_c;
                    tt_wd <= "000000" & std_logic_vector(t_ey(17 downto 8)) & std_logic_vector(t_ec(1)(15 downto 8)) & std_logic_vector(t_ec(2)(15 downto 8));
                    if t_c = 14 then
                        t_ey <= shift_left(resize(t_cy, 20), 4);
                        for i in 1 to 2 loop t_ec(i) <= shift_left(resize(t_cc(i), 18), 4); end loop;
                    else
                        t_ey <= t_ey + resize(t_sy, 20);
                        for i in 1 to 2 loop t_ec(i) <= t_ec(i) + resize(t_sc(i), 18); end loop;
                    end if;
                    t_c <= t_c + 1;
                    if t_c = 15 then
                        t_row <= '1';
                        if t_g(3) = '0' then
                            t_cy <= t_cy + signed(resize(c_by & '0', 16)) - signed(resize(c_ay & '0', 16));
                            for i in 1 to 2 loop t_cc(i) <= t_cc(i) + signed(resize(c_bc(i) & '0', 14)) - signed(resize(c_ac(i) & '0', 14)); end loop;
                        else
                            t_cy <= t_cy + signed(resize(c_ky & '0', 16)) - signed(resize(c_by & '0', 16));
                            for i in 1 to 2 loop t_cc(i) <= t_cc(i) + signed(resize(c_kc(i) & '0', 14)) - signed(resize(c_bc(i) & '0', 14)); end loop;
                        end if;
                        t_g <= t_g + 1;
                        if t_g = 15 then t_run <= '0'; end if;
                    end if;
                end if;
            end if;
        end if;
    end process p_fill;

    ---------------------------------------------------------------------------
    -- [D] Wave engine: the wave table, refilled every field (from zc 560, 12
    -- clocks per entry).  Entry j (signed half-cell index -128 .. 127) =
    -- 280 +- Depth * wave(j * Spread / 2 - phase), with the cell's colour step.
    -- The same ROM gives the lens centre's two sines at the top of the zone.
    ---------------------------------------------------------------------------
    p_wfill : process(clk)
        variable v_th  : unsigned(13 downto 0);
        variable v_pos : unsigned(11 downto 0);
        variable v_q   : unsigned(8 downto 0);
        variable v_gi  : unsigned(3 downto 0);
        variable v_fb  : unsigned(9 downto 0);
        variable v_sm  : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            wt_we <= '0';
            if s_zone = '1' and s_zc = 4 then
                v_q := s_wx(13 downto 5);
                if s_wx(14) = '1' then v_q := not v_q; end if;
                r_w_a <= v_q;
            elsif s_zone = '1' and s_zc = 7 then
                v_q := s_wy(13 downto 5);
                if s_wy(14) = '1' then v_q := not v_q; end if;
                r_w_a <= v_q;
            elsif s_zone = '1' and s_zc = 556 then
                w_acc <= 0 - shift_left(resize(c_s, 24), 6);
            elsif s_zone = '1' and s_zc = 557 then
                w_acc <= w_acc - c_ph(25 downto 2);
                w_j <= x"80";
            elsif s_zone = '1' and s_zc = 560 then
                w_run <= '1'; w_ph <= (others => '0');
            elsif w_run = '1' then
                w_ph <= w_ph + 1;
                case to_integer(w_ph) is
                    when 0 =>
                        v_th := w_acc(23 downto 10);
                        v_pos := v_th(11 downto 0);
                        if v_th(12) = '1' then v_pos := not v_pos; end if;
                        if s_snap = '0' then r_w_a <= v_pos(11 downto 3);
                        elsif v_pos(11 downto 10) /= "00" then r_w_a <= (others => '1');
                        else r_w_a <= v_pos(9 downto 1); end if;
                        w_neg <= v_th(13);
                    when 1 => null;
                    when 2 =>
                        w_mb <= c_d; w_mc <= (others => '0');
                    when 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 =>       -- (the ROM word stays put: Depth * wave / 256, exact)
                        v_sm := resize(w_mc, 11);
                        if w_mb(0) = '1' then v_sm := v_sm + resize(r_w_q(9 downto 0), 11); end if;
                        w_mc <= v_sm(10 downto 1);
                        w_mb <= '0' & w_mb(7 downto 1);
                    when others =>
                        if w_neg = '1' then v_fb := to_unsigned(280, 10) - resize(w_mc(9 downto 2), 10);
                        else v_fb := to_unsigned(280, 10) + resize(w_mc(9 downto 2), 10); end if;
                        if w_j(5) = '1' then v_gi := not w_j(4 downto 1); else v_gi := w_j(4 downto 1); end if;
                        wt_we <= '1'; wt_wa <= w_j;
                        wt_wd <= std_logic_vector(v_gi) & "00" & std_logic_vector(v_fb);
                        w_acc <= w_acc + resize(c_s(22 downto 1), 24);
                        w_j <= w_j + 1;
                        w_ph <= (others => '0');
                        if w_j = x"7F" then w_run <= '0'; s_ok <= '1'; end if;
                end case;
            end if;
        end if;
    end process p_wfill;

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
    -- px) drifting diagonally.  The luma (or the lattice) is smeared along x
    -- by a 1/16 filter and scaled into the size term (Video x1.5, Osc x2); the
    -- per-cell hold is in p_cell.
    ---------------------------------------------------------------------------
    p_src : process(clk)
        variable v_tx, v_ty : unsigned(6 downto 0);
        variable v_s  : unsigned(7 downto 0);
        variable v_o  : unsigned(7 downto 0);
        variable v_l  : unsigned(7 downto 0);
        variable v_x  : unsigned(13 downto 0);
        variable v_d1 : signed(14 downto 0);
        variable v_cs : unsigned(15 downto 0);
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

            -- input sample (8 bits), held through blanking
            s_sav <= data_in.avid; s_savd <= s_sav;
            if data_in.avid = '1' then
                if s_src = '1' then
                    s_sv <= v_o;
                else
                    v_l := unsigned(data_in.y(9 downto 2));
                    if v_l > 16 then s_sv <= v_l - 16; else s_sv <= (others => '0'); end if;
                end if;
            end if;

            -- smear; it runs 40 pixels past the line end
            if s_sav = '0' and s_savd = '1' then
                s_ext <= to_unsigned(40, 6);
            elsif s_ext /= 0 then
                s_ext <= s_ext - 1;
            end if;
            v_x := s_sv & "000000";
            if s_sav = '1' and s_savd = '0' then
                s_y1 <= v_x;
            elsif s_sav = '1' or s_ext /= 0 then
                v_d1 := signed(resize(v_x, 15)) - signed(resize(s_y1, 15));
                s_y1 <= unsigned(signed(s_y1) + resize(shift_right(v_d1, 4), 14));
            end if;
            -- K6 camera pass: the size term = smear * c_cam / 32 (two gated partial sums, then their sum; one
            -- clock later than v0.1.0's x1.5 / x2, which c_cam = 48 / 64 reproduces exactly)
            s_cpa <= f_gu(s_y1(13 downto 6), c_cam(0), 12) + shift_left(f_gu(s_y1(13 downto 6), c_cam(1), 12), 1)
                   + shift_left(f_gu(s_y1(13 downto 6), c_cam(2), 12), 2) + shift_left(f_gu(s_y1(13 downto 6), c_cam(3), 12), 3);
            s_cpb <= f_gu(s_y1(13 downto 6), c_cam(4), 11) + shift_left(f_gu(s_y1(13 downto 6), c_cam(5), 11), 1)
                   + shift_left(f_gu(s_y1(13 downto 6), c_cam(6), 11), 2);
            v_cs := resize(s_cpa, 16) + shift_left(resize(s_cpb, 16), 4);
            s_st <= v_cs(14 downto 5);
        end if;
    end process p_src;

    ---------------------------------------------------------------------------
    -- [E] Lens.  Stage 1: accumulators (u in norm px Q16; dx^2 in half virtual
    -- px by second differences).  2: r^2 -> the lens and ramp ROM addresses.
    -- 5 .. 7: m = Bulge * h.  8 .. 11: (u - centre) * m, (v - centre) * m.
    -- 12: u', v'.  13: cell id and |sub-cell coordinate|.
    ---------------------------------------------------------------------------
    p_acc : process(clk)
    begin
        if rising_edge(clk) then
            -- the line registers are loaded straight from the zone's products
            if s_zone = '1' and s_zc = 252 then
                ls_vn <= c_hc - signed(resize(f_acc(26 downto 0), 28));
            elsif g_start = '1' then
                ls_vn <= ls_vn + signed(resize(c_sy, 28));
            end if;
            if s_zone = '1' and s_zc = 348 then
                ls_qy <= signed(resize(f_acc(29 downto 0), 31));
            elsif g_start = '1' then
                ls_qy <= ls_qy + resize(ls_iy, 31);
            end if;
            if s_zone = '1' and s_zc = 380 then
                ls_iy <= to_signed(0, 27) - signed(resize(f_acc(23 downto 0) & '0', 27));
            elsif s_zone = '1' and s_zc = 412 then
                ls_iy <= ls_iy + signed(resize(f_acc(17 downto 0), 27));
            elsif g_start = '1' then
                ls_iy <= ls_iy + signed(resize(c_iyy, 27));
            end if;
            if g_start = '1' then
                a_un <= c_un0 & "0000000000000"; a_qx <= signed(resize(c_qx0, 25)); a_ix <= c_ix0;
                l_vnq <= ls_vn(27 downto 13); l_qy8 <= unsigned(ls_qy(28 downto 8));
            elsif g_avid = '1' then
                a_un <= a_un + signed(resize(c_sx, 28));
                a_qx <= a_qx + resize(a_ix, 25);
                a_ix <= a_ix + signed(resize(c_ixx, 18));
            end if;
            l_dy <= l_vnq(13 downto 1) - c_cvq;
        end if;
    end process p_acc;

    p_lens : process(clk)
        variable v_r2 : unsigned(24 downto 0);
        variable v_p  : unsigned(17 downto 0);
        variable v_pr : signed(22 downto 0);
        variable v_u  : signed(14 downto 0);
        variable v_lo, v_hf : unsigned(9 downto 0);
        variable v_tp : std_logic;
    begin
        if rising_edge(clk) then
            -- 2
            q2_un <= a_un(27 downto 13);
            q2_dx <= a_un(26 downto 14) - c_cuq;
            v_r2 := unsigned(a_qx) + resize(l_qy8, 25);
            r_h_a <= s_pin & v_r2(19 downto 10);
            r_l_a <= s_pin & c_b8(7 downto 5) & v_r2(19 downto 14);
            if v_r2(24 downto 20) = 0 then q2_in <= '1'; else q2_in <= '0'; end if;
            -- 3: ROMs; 4: fabric copies, zero outside the lens
            q3_in <= q2_in;
            if q3_in = '1' then
                q4_h <= r_h_q(11 downto 2); q4_l <= signed(r_l_q(4 downto 0));
            else
                q4_h <= (others => '0'); q4_l <= (others => '0');
            end if;
            -- 5 .. 7: m = Bulge * h (gated sums); the ramp exponent
            q5_p0 <= f_gu(q4_h, c_b8(0), 12) + shift_left(f_gu(q4_h, c_b8(1), 12), 1);
            q5_p1 <= f_gu(q4_h, c_b8(2), 12) + shift_left(f_gu(q4_h, c_b8(3), 12), 1);
            q5_p2 <= f_gu(q4_h, c_b8(4), 12) + shift_left(f_gu(q4_h, c_b8(5), 12), 1);
            q5_p3 <= f_gu(q4_h, c_b8(6), 12) + shift_left(f_gu(q4_h, c_b8(7), 12), 1);
            q5_l4 <= q4_l + signed(resize(c_lf, 5));
            q6_qa <= resize(q5_p0, 14) + shift_left(resize(q5_p1, 14), 2);
            q6_qb <= resize(q5_p2, 14) + shift_left(resize(q5_p3, 14), 2);
            v_p := resize(q6_qa, 18) + shift_left(resize(q6_qb, 18), 4);
            q7_m <= v_p(17 downto 8);
            q7_dx <= signed(dr_q(12 downto 0));
            -- 8 .. 11: the two lens products (13 x 10 bits)
            for i in 0 to 4 loop
                q8_px(i) <= f_gs(q7_dx, q7_m(2 * i), 15) + shift_left(f_gs(q7_dx, q7_m(2 * i + 1), 15), 1);
                q8_py(i) <= f_gs(l_dy, q7_m(2 * i), 15) + shift_left(f_gs(l_dy, q7_m(2 * i + 1), 15), 1);
            end loop;
            q9_ax <= resize(q8_px(0), 17) + shift_left(resize(q8_px(1), 17), 2);
            q9_bx <= resize(q8_px(2), 17) + shift_left(resize(q8_px(3), 17), 2);
            q9_cx <= q8_px(4);
            q9_ay <= resize(q8_py(0), 17) + shift_left(resize(q8_py(1), 17), 2);
            q9_by <= resize(q8_py(2), 17) + shift_left(resize(q8_py(3), 17), 2);
            q9_cy <= q8_py(4);
            q10_tx <= resize(q9_ax, 21) + shift_left(resize(q9_bx, 21), 4);
            q10_ux <= q9_cx;
            q10_ty <= resize(q9_ay, 21) + shift_left(resize(q9_by, 21), 4);
            q10_uy <= q9_cy;
            v_pr := resize(q10_tx, 23) + shift_left(resize(q10_ux, 23), 8);
            q11_du <= v_pr(22 downto 9);
            v_pr := resize(q10_ty, 23) + shift_left(resize(q10_uy, 23), 8);
            q11_dv <= v_pr(22 downto 9);
            q11_un <= signed(ur_q(14 downto 0));
            -- 12: u' = u - d m (Bulge) or u + 2 d m (Pinch)
            if s_pin = '1' then
                q12_u <= q11_un + shift_left(resize(q11_du, 15), 1);
                q12_v <= l_vnq + shift_left(resize(q11_dv, 15), 1);
            else
                q12_u <= q11_un - resize(q11_du, 15);
                q12_v <= l_vnq - resize(q11_dv, 15);
            end if;
            -- 13: cell id (high bits) and |sub-cell coordinate| (low bits), in 1/8 norm px
            for ax in 0 to 1 loop
                if ax = 0 then v_u := q12_u; else v_u := q12_v; end if;
                case c_sh is
                    when "00"   => v_lo := resize(unsigned(v_u(5 downto 0)), 10); v_tp := v_u(6); v_hf := to_unsigned(64, 10);
                    when "01"   => v_lo := resize(unsigned(v_u(6 downto 0)), 10); v_tp := v_u(7); v_hf := to_unsigned(128, 10);
                    when "10"   => v_lo := resize(unsigned(v_u(7 downto 0)), 10); v_tp := v_u(8); v_hf := to_unsigned(256, 10);
                    when others => v_lo := resize(unsigned(v_u(8 downto 0)), 10); v_tp := v_u(9); v_hf := to_unsigned(512, 10);
                end case;
                if v_tp = '0' then v_lo := v_hf - v_lo; end if;
                if ax = 0 then
                    q13_a <= v_lo;
                    case c_sh is
                        when "00"   => q13_ci <= v_u(14 downto 7);
                        when "01"   => q13_ci <= resize(v_u(14 downto 8), 8);
                        when "10"   => q13_ci <= resize(v_u(14 downto 9), 8);
                        when others => q13_ci <= resize(v_u(14 downto 10), 8);
                    end case;
                else
                    q13_b <= v_lo;
                    case c_sh is
                        when "00"   => q13_cj <= v_u(14 downto 7);
                        when "01"   => q13_cj <= resize(v_u(14 downto 8), 8);
                        when "10"   => q13_cj <= resize(v_u(14 downto 9), 8);
                        when others => q13_cj <= resize(v_u(14 downto 10), 8);
                    end case;
                end if;
            end loop;
        end if;
    end process p_lens;

    ---------------------------------------------------------------------------
    -- Cells: 14 .. 16 the wave table by index and the per-cell source hold;
    -- 17, 18 size = table + source; 14 .. 19 the shape metric.
    ---------------------------------------------------------------------------
    p_cell : process(clk)
        variable v_f  : unsigned(10 downto 0);
        variable v_fu : unsigned(8 downto 0);
        variable v_tm : std_logic;
        variable v_ai, v_aj : unsigned(7 downto 0);
        variable v_sum : signed(7 downto 0);
        variable v_mr : unsigned(10 downto 0);
        variable v_sj : unsigned(14 downto 0);
        variable v_d45 : unsigned(10 downto 0);
    begin
        if rising_edge(clk) then
            -- 14: sort; index in half cells -> wave table address; the hold's read address
            if q13_a >= q13_b then q14_hi <= q13_a; q14_lo <= q13_b; else q14_hi <= q13_b; q14_lo <= q13_a; end if;
            v_ai := unsigned(abs(q13_ci)); v_aj := unsigned(abs(q13_cj));
            v_sum := q13_ci + q13_cj;
            if s_rad = '0' then
                wt_ra <= unsigned(v_sum(6 downto 0)) & '0';
            elsif v_ai >= v_aj then
                wt_ra <= (v_ai(6 downto 0) & '0') + v_aj;
            else
                wt_ra <= (v_aj(6 downto 0) & '0') + v_ai;
            end if;
            sh_ra <= q13_cj(0) & unsigned(q13_ci(6 downto 0));
            q14_id <= std_logic_vector(q13_cj(5 downto 0)) & std_logic_vector(q13_ci(6 downto 0));
            -- 15: RAMs; a new cell starts on this line
            q15_id <= q14_id;
            if s_gav_sr(12) = '1' and (s_gav_sr(13) = '0' or q14_id /= q15_id) then q15_new <= '1'; else q15_new <= '0'; end if;
            -- 16: fabric copies
            q16_fb <= unsigned(wt_q(9 downto 0)); q16_gi <= unsigned(wt_q(15 downto 12));
            q16_hq <= sh_q; q16_id <= q15_id; q16_new <= q15_new;
            -- 17: the cell's source term: the held one if this cell row wrote it, else the smeared source now
            -- (and store it)
            if q16_hq(15 downto 10) = (s_ot(0) & q16_id(12 downto 8)) then v_tm := '1'; else v_tm := '0'; end if;
            sh_we <= '0';
            if q16_new = '1' then
                if v_tm = '1' then
                    q17_st <= unsigned(q16_hq(9 downto 0));
                else
                    q17_st <= s_st;
                    sh_we <= '1';
                end if;
            end if;
            sh_wa <= unsigned(q16_id(7 downto 0));
            sh_wd <= s_ot(0) & q16_id(12 downto 8) & std_logic_vector(s_st);
            q17_fb <= q16_fb;
            -- 18: size (of 512 = the half cell), clamped, in 1/8 norm px
            v_f := resize(q17_fb, 11) + resize(q17_st, 11);
            if v_f < 56 then v_fu := to_unsigned(56, 9);
            elsif v_f > 472 then v_fu := to_unsigned(472, 9);
            else v_fu := v_f(8 downto 0); end if;
            case c_sh is
                when "00"   => q18_s <= resize(v_fu(8 downto 3), 10);
                when "01"   => q18_s <= resize(v_fu(8 downto 2), 10);
                when "10"   => q18_s <= resize(v_fu(8 downto 1), 10);
                when others => q18_s <= resize(v_fu, 10);
            end case;
            q19_s <= q18_s;

            -- shape metric ----------------------------------------------------------------------------------
            -- 15: the 16-gon's 22.5-degree face distance by shift-adds (hi and lo parts)
            q15_sm  <= resize(q14_hi, 11) + resize(q14_lo, 11);
            q15_hi  <= q14_hi;
            q15_x22 <= q14_hi - shift_right(q14_hi, 4) - shift_right(q14_hi, 6);
            q15_y22 <= shift_right(q14_lo, 2) + shift_right(q14_lo, 3) + shift_right(q14_lo, 7);
            -- 16: the face distance; (hi + lo) * chamfer coefficient
            q16_d22 <= resize(q15_x22, 11) + resize(q15_y22, 11);
            q16_ja  <= f_gu(q15_sm, c_j4(0), 13) + shift_left(f_gu(q15_sm, c_j4(1), 13), 1);
            q16_jb  <= f_gu(q15_sm, c_j4(2), 13) + shift_left(f_gu(q15_sm, c_j4(3), 13), 1);
            q16_sm  <= q15_sm; q16_hi <= q15_hi;
            -- 17: the chamfer distance
            q17_mr <= q16_d22;
            v_sj := resize(q16_ja, 15) + shift_left(resize(q16_jb, 15), 2);
            q17_d45 <= resize(q16_sm(10 downto 1), 11) + resize(v_sj(14 downto 6), 11);
            q17_hi <= q16_hi;
            -- 18: the octagon metric; the 16-gon's shrink t / 128 (it only cuts the octagon's corners once Shape
            -- passes the octagon)
            q18_m8 <= f_max(resize(q17_hi, 11), q17_d45);
            q18_pt <= (f_gu(q17_mr, c_t4(0), 15) + shift_left(f_gu(q17_mr, c_t4(1), 15), 1))
                    + (shift_left(f_gu(q17_mr, c_t4(2), 15), 2) + shift_left(f_gu(q17_mr, c_t4(3), 15), 3));
            q18_mr <= q17_mr;
            -- 19: the metric
            if c_rnd = '1' then v_mr := q18_mr - resize(q18_pt(14 downto 7), 11); else v_mr := (others => '0'); end if;
            q19_d <= f_max(q18_m8, v_mr);
        end if;
    end process p_cell;

    ---------------------------------------------------------------------------
    -- Coverage and colour: 20 metric - size; 21 x the ramp mantissa; 22 the
    -- ramp's power of two; 23 coverage in sixteenths; 24 .. 26 the blend table.
    ---------------------------------------------------------------------------
    p_look : process(clk)
        variable v_e  : signed(11 downto 0);
        variable v_e15, v_t1, v_t2, v_t3 : signed(14 downto 0);
        variable v_dl : signed(14 downto 0);
        variable v_c  : signed(6 downto 0);
        variable v_gi : unsigned(3 downto 0);
    begin
        if rising_edge(clk) then
            -- 20
            v_e := signed(resize(q19_d, 12)) - signed(resize(q19_s, 12));
            if v_e > 255 then q20_e <= to_signed(255, 9);
            elsif v_e < -255 then q20_e <= to_signed(-255, 9);
            else q20_e <= v_e(8 downto 0); end if;
            -- 21: x 16, 19, 23, 27 (quarter-octave mantissa of the ramp slope; lr_f = the ramp exponent, stage 20)
            v_e15 := resize(q20_e, 15);
            case lr_f(1 downto 0) is
                when "00"   => v_t1 := shift_left(v_e15, 4); v_t2 := (others => '0');       v_t3 := (others => '0');
                when "01"   => v_t1 := shift_left(v_e15, 4); v_t2 := shift_left(v_e15, 1);  v_t3 := v_e15;
                when "10"   => v_t1 := shift_left(v_e15, 4); v_t2 := shift_left(v_e15, 3);  v_t3 := -v_e15;
                when others => v_t1 := shift_left(v_e15, 5); v_t2 := -shift_left(v_e15, 2); v_t3 := -v_e15;
            end case;
            q21_v <= v_t1 + v_t2 + v_t3;
            q21_ex <= signed(lr_f(4 downto 2));
            -- 22: >> (4 - exponent), saturated
            case to_integer(q21_ex) is
                when 3      => v_dl := shift_right(q21_v, 1);
                when 2      => v_dl := shift_right(q21_v, 2);
                when 1      => v_dl := shift_right(q21_v, 3);
                when 0      => v_dl := shift_right(q21_v, 4);
                when -1     => v_dl := shift_right(q21_v, 5);
                when -2     => v_dl := shift_right(q21_v, 6);
                when -3     => v_dl := shift_right(q21_v, 7);
                when others => v_dl := shift_right(q21_v, 8);
            end case;
            if v_dl > 31 then q22_dl <= to_signed(31, 6);
            elsif v_dl < -32 then q22_dl <= to_signed(-32, 6);
            else q22_dl <= v_dl(5 downto 0); end if;
            -- 23: coverage 0 .. 15 (15 = full)
            v_c := to_signed(8, 7) - resize(q22_dl, 7);
            if v_c < 0 then q23_cov <= (others => '0');
            elsif v_c > 15 then q23_cov <= (others => '1');
            else q23_cov <= unsigned(v_c(3 downto 0)); end if;
            -- 24: blend table address (gr_f = the colour step, stage 23); 25: RAM; 26: the pixel (the ground
            -- colour outside the generated avid, neutral until the first field's tables are filled)
            if s_grade = '1' then v_gi := unsigned(gr_f(3 downto 0)); else v_gi := (others => '0'); end if;
            tt_ra <= v_gi & q23_cov;
            if s_ok = '0' then
                q29_y <= to_unsigned(64, 10); q29_c(1) <= to_unsigned(128, 8); q29_c(2) <= to_unsigned(128, 8);
            elsif s_gav_sr(23) = '0' then
                q29_y <= c_gy; q29_c <= c_gc;
            else
                q29_y <= unsigned(tt_q(25 downto 16));
                q29_c(1) <= unsigned(tt_q(15 downto 8)); q29_c(2) <= unsigned(tt_q(7 downto 0));
            end if;
            s_bah <= s_bah(6 downto 0) & s_gav_sr(23);
        end if;
    end process p_look;

    ---------------------------------------------------------------------------
    -- [F] Output conditioning: chroma [1 2 2 2 2 2 2 2 1]/16 (two adjacent
    -- 8-sums of the 8-bit chroma, resynced every line: 10 bits out), luma [1 2 1]/4 delayed to the chroma's
    -- group delay, luma 13/16, blanking gate.  The filters add 4 pixels of
    -- delay: the output register is effective stage 34 = C_LAT + 1.
    ---------------------------------------------------------------------------
    p_out : process(clk)
        variable v_sc : unsigned(11 downto 0);
        variable v_y  : unsigned(11 downto 0);
        variable v_dl : unsigned(9 downto 0);
    begin
        if rising_edge(clk) then
            -- 27: running 8-sums (the sample of 8 clocks before comes back from its delay ring); they snap
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
            -- 28: filtered
            v_sc := resize(s_bu, 12) + s_bud; s_u31 <= v_sc(11 downto 2);
            v_sc := resize(s_bv, 12) + s_bvd; s_v31 <= v_sc(11 downto 2);
            v_y := resize(s_ysr(3), 12) + resize(s_ysr(4) & '0', 12) + resize(s_ysr(5), 12);
            s_y31 <= v_y(11 downto 2);
            -- 29: luma 13/16 toward black
            if s_y31 > 64 then v_dl := s_y31 - 64; else v_dl := (others => '0'); end if;
            s_y32 <= s_y31 - shift_right(v_dl, 3) - shift_right(v_dl, 4);
            s_u32 <= s_u31; s_v32 <= s_v31;
            -- 30: blanking gate
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

end architecture vega;
